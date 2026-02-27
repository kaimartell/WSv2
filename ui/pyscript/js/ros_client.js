(function () {
  const DEFAULT_URL = "ws://localhost:9090";
  const QUICK_FAIL_WINDOW_MS = 1500;
  const DISCONNECT_GUARD_MS = 500;

  const ROSAPI_TYPES = {
    topics: "rosapi_msgs/srv/Topics",
    nodes: "rosapi_msgs/srv/Nodes",
    services: "rosapi_msgs/srv/Services",
    topicType: "rosapi_msgs/srv/TopicType",
    serviceType: "rosapi_msgs/srv/ServiceType",
    publishers: "rosapi_msgs/srv/Publishers",
    subscribers: "rosapi_msgs/srv/Subscribers",
    nodeDetails: "rosapi_msgs/srv/NodeDetails",
  };

  const state = {
    ros: null,
    phase: "idle",
    connected: false,
    requestedUrl: "",
    activeUrl: "",
    manualDisconnect: false,
    lastError: "",
    attemptSeq: 0,
    activeAttempt: null,
    autoFallbackUsed: false,
    statusTopic: null,
    statusCallback: null,
    logTopic: null,
    logConfig: null,
    logCallback: null,
    topicSubscriptions: new Map(),
    publisherCache: new Map(),
    callbacks: {
      connectionState: null,
      error: null,
      debug: null,
    },
  };

  function safeStringify(value) {
    try {
      if (typeof value === "string") {
        return value;
      }
      return JSON.stringify(value);
    } catch (_err) {
      try {
        return String(value);
      } catch (_err2) {
        return "<unprintable>";
      }
    }
  }

  function formatError(err) {
    if (err == null) {
      return "unknown error";
    }
    if (typeof err === "string") {
      return err;
    }
    if (err.message) {
      return String(err.message);
    }
    return safeStringify(err);
  }

  function emitDebug(message) {
    if (typeof state.callbacks.debug !== "function") {
      return;
    }
    try {
      state.callbacks.debug(String(message));
    } catch (cbErr) {
      console.error("[rosClient] debug callback failed", cbErr);
    }
  }

  function consoleLog(level, message, details) {
    const prefix = "[rosClient]";
    if (details !== undefined) {
      (console[level] || console.log).call(console, prefix, message, details);
      emitDebug(`${message} ${safeStringify(details)}`);
    } else {
      (console[level] || console.log).call(console, prefix, message);
      emitDebug(String(message));
    }
  }

  function emitError(message) {
    state.lastError = String(message || "").trim();
    if (typeof state.callbacks.error === "function") {
      try {
        state.callbacks.error(state.lastError);
      } catch (cbErr) {
        console.error("[rosClient] error callback failed", cbErr);
      }
    }
  }

  function emitConnectionState(extra) {
    const payload = {
      phase: state.phase,
      connected: state.connected,
      requestedUrl: state.requestedUrl,
      url: state.activeUrl,
      lastError: state.lastError,
      autoFallbackUsed: state.autoFallbackUsed,
      message: "",
      fallbackFrom: "",
      ...extra,
    };
    if (typeof state.callbacks.connectionState === "function") {
      try {
        state.callbacks.connectionState(JSON.stringify(payload));
      } catch (cbErr) {
        console.error("[rosClient] connectionState callback failed", cbErr);
      }
    }
  }

  function parseUrlMaybe(url) {
    try {
      return new URL(url);
    } catch (_err) {
      return null;
    }
  }

  function normalizeUrl(url) {
    return String(url || "").trim() || DEFAULT_URL;
  }

  function computeLocalhostIpv4Fallback(url) {
    const parsed = parseUrlMaybe(url);
    if (!parsed || parsed.hostname !== "localhost") {
      return null;
    }
    parsed.hostname = "127.0.0.1";
    return parsed.toString();
  }

  function unsubscribeRosTopic(topicObj) {
    if (!topicObj) {
      return;
    }
    try {
      topicObj.unsubscribe();
    } catch (err) {
      console.warn("[rosClient] topic unsubscribe failed", err);
    }
  }

  function unsubscribeAllTopicSubscriptions() {
    for (const [topicName, entry] of state.topicSubscriptions.entries()) {
      unsubscribeRosTopic(entry && entry.topicObj);
      consoleLog("info", "Unsubscribed dynamic topic", { topicName });
    }
    state.topicSubscriptions.clear();
  }

  function teardownTransport(options) {
    const opts = { closeSocket: false, ...options };

    unsubscribeRosTopic(state.statusTopic);
    unsubscribeRosTopic(state.logTopic);
    state.statusTopic = null;
    state.logTopic = null;
    unsubscribeAllTopicSubscriptions();
    state.publisherCache.clear();

    const rosToClose = state.ros;
    state.ros = null;
    state.connected = false;

    if (opts.closeSocket && rosToClose) {
      try {
        rosToClose.close();
      } catch (err) {
        console.warn("[rosClient] ros.close() during teardown failed", err);
      }
    }
  }

  function isCurrentAttempt(attemptId) {
    return !!state.activeAttempt && state.activeAttempt.id === attemptId;
  }

  function getPublisherKey(topicName, messageType) {
    return `${String(topicName)}|${String(messageType)}`;
  }

  function ensurePublisher(topicName, messageType) {
    if (!state.connected || !state.ros) {
      return null;
    }
    const name = String(topicName || "").trim();
    const type = String(messageType || "").trim();
    if (!name || !type) {
      return null;
    }
    const key = getPublisherKey(name, type);
    if (!state.publisherCache.has(key)) {
      state.publisherCache.set(
        key,
        new ROSLIB.Topic({
          ros: state.ros,
          name,
          messageType: type,
        })
      );
      consoleLog("info", "Created publisher", { topic: name, messageType: type });
    }
    return state.publisherCache.get(key);
  }

  function attachStatusTopicIfNeeded() {
    unsubscribeRosTopic(state.statusTopic);
    state.statusTopic = null;
    if (!state.connected || !state.ros || typeof state.statusCallback !== "function") {
      return;
    }
    state.statusTopic = new ROSLIB.Topic({
      ros: state.ros,
      name: "/status",
      messageType: "std_msgs/String",
    });
    state.statusTopic.subscribe((msg) => {
      try {
        const text = msg && typeof msg.data === "string" ? msg.data : safeStringify(msg);
        state.statusCallback(text);
      } catch (cbErr) {
        console.error("[rosClient] status callback failed", cbErr);
      }
    });
    consoleLog("info", "Subscribed to /status");
  }

  function attachLogTopicIfNeeded() {
    unsubscribeRosTopic(state.logTopic);
    state.logTopic = null;
    if (!state.connected || !state.ros || !state.logConfig || typeof state.logCallback !== "function") {
      return;
    }
    const { topicName, messageType } = state.logConfig;
    state.logTopic = new ROSLIB.Topic({
      ros: state.ros,
      name: topicName,
      messageType,
    });
    state.logTopic.subscribe((msg) => {
      try {
        if (messageType === "rcl_interfaces/Log") {
          const levelMap = { 10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL" };
          const level = levelMap[msg.level] || String(msg.level ?? "");
          const name = msg.name || "rosout";
          const text = msg.msg || safeStringify(msg);
          state.logCallback(`[${level}] ${name}: ${text}`);
          return;
        }
        if (messageType === "std_msgs/String") {
          state.logCallback(typeof msg.data === "string" ? msg.data : safeStringify(msg));
          return;
        }
        state.logCallback(safeStringify(msg));
      } catch (cbErr) {
        console.error("[rosClient] log callback failed", cbErr);
      }
    });
    consoleLog("info", `Subscribed to ${topicName}`, { messageType });
  }

  function maybeFallbackFromAttempt(attempt, reason, errorText) {
    if (!attempt || attempt.connected || attempt.fallbackTried || state.manualDisconnect) {
      return false;
    }
    const elapsedMs = Date.now() - attempt.startedAtMs;
    const fallbackUrl = computeLocalhostIpv4Fallback(attempt.url);
    if (!fallbackUrl || elapsedMs > QUICK_FAIL_WINDOW_MS) {
      consoleLog("info", "Not using fallback", { reason, elapsedMs, fallbackUrl });
      return false;
    }

    attempt.fallbackTried = true;
    state.autoFallbackUsed = true;
    consoleLog("warn", "Quick connection failure, retrying with IPv4 localhost fallback", {
      reason,
      from: attempt.url,
      to: fallbackUrl,
      elapsedMs,
      error: errorText,
    });

    emitConnectionState({
      phase: "connecting",
      connected: false,
      url: fallbackUrl,
      fallbackFrom: attempt.url,
      autoFallbackUsed: true,
      message: `Retrying with ${fallbackUrl} (localhost fallback)`,
    });

    teardownTransport({ closeSocket: true });
    state.activeAttempt = null;
    startAttempt(fallbackUrl);
    return true;
  }

  function finalizeDisconnected(message) {
    state.phase = "idle";
    state.connected = false;
    state.activeUrl = "";
    state.activeAttempt = null;
    emitConnectionState({ phase: "idle", connected: false, message: message || "Disconnected" });
  }

  function startAttempt(url) {
    if (typeof ROSLIB === "undefined") {
      const msg = "ROSLIB is not loaded";
      emitError(msg);
      state.phase = "error";
      emitConnectionState({ phase: "error", connected: false, url, message: msg });
      console.error("[rosClient]", msg);
      return false;
    }

    const attempt = {
      id: ++state.attemptSeq,
      url,
      startedAtMs: Date.now(),
      connected: false,
      fallbackTried: false,
    };

    state.activeAttempt = attempt;
    state.activeUrl = url;
    state.phase = "connecting";

    emitConnectionState({ phase: "connecting", connected: false, url, message: `Connecting to ${url} ...` });
    consoleLog("info", "Starting rosbridge connection attempt", { attemptId: attempt.id, url });

    let ros;
    try {
      ros = new ROSLIB.Ros({ url });
    } catch (err) {
      const msg = formatError(err);
      emitError(msg);
      state.phase = "error";
      emitConnectionState({ phase: "error", connected: false, url, message: `Connection setup error: ${msg}` });
      consoleLog("error", "ROSLIB.Ros constructor failed", { url, error: msg });
      return false;
    }

    state.ros = ros;

    ros.on("connection", () => {
      if (!isCurrentAttempt(attempt.id)) {
        consoleLog("info", "Ignoring stale connection event", { attemptId: attempt.id });
        return;
      }
      attempt.connected = true;
      state.connected = true;
      state.phase = "connected";
      state.activeUrl = attempt.url;
      state.lastError = "";
      attachStatusTopicIfNeeded();
      attachLogTopicIfNeeded();
      consoleLog("info", "Rosbridge connected", { attemptId: attempt.id, url: attempt.url });
      emitConnectionState({
        phase: "connected",
        connected: true,
        url: attempt.url,
        autoFallbackUsed: state.autoFallbackUsed,
        message: `Connected to ${attempt.url}`,
      });
    });

    ros.on("error", (err) => {
      if (!isCurrentAttempt(attempt.id)) {
        consoleLog("info", "Ignoring stale error event", { attemptId: attempt.id });
        return;
      }
      const msg = formatError(err);
      emitError(msg);
      consoleLog("error", "Rosbridge error event", { attemptId: attempt.id, url: attempt.url, error: msg });
      if (!attempt.connected) {
        if (maybeFallbackFromAttempt(attempt, "error", msg)) {
          return;
        }
        state.phase = "error";
        emitConnectionState({ phase: "error", connected: false, url: attempt.url, message: `Connection error: ${msg}` });
      }
    });

    ros.on("close", () => {
      if (!isCurrentAttempt(attempt.id)) {
        consoleLog("info", "Ignoring stale close event", { attemptId: attempt.id });
        return;
      }
      consoleLog("warn", "Rosbridge close event", {
        attemptId: attempt.id,
        url: attempt.url,
        connectedOnce: attempt.connected,
        manualDisconnect: state.manualDisconnect,
      });
      if (!attempt.connected && maybeFallbackFromAttempt(attempt, "close", state.lastError || "connection closed")) {
        return;
      }
      teardownTransport({ closeSocket: false });
      finalizeDisconnected(state.manualDisconnect ? "Disconnected" : "Connection closed");
    });

    return true;
  }

  function callServiceObject(serviceName, serviceType, requestObj, callback) {
    const name = String(serviceName || "").trim();
    const type = String(serviceType || "").trim();
    const request = requestObj && typeof requestObj === "object" ? requestObj : {};

    consoleLog("info", "callServiceObject()", { serviceName: name, serviceType: type, request });

    if (typeof callback !== "function") {
      consoleLog("warn", "callServiceObject() missing callback");
      return false;
    }
    if (!state.connected || !state.ros) {
      const msg = "Cannot call service: not connected.";
      emitError(msg);
      try { callback(null, msg); } catch (_err) {}
      return false;
    }
    if (!name || !type) {
      const msg = "Service name and type are required.";
      emitError(msg);
      try { callback(null, msg); } catch (_err) {}
      return false;
    }

    try {
      const service = new ROSLIB.Service({ ros: state.ros, name, serviceType: type });
      const req = new ROSLIB.ServiceRequest(request);
      service.callService(
        req,
        (response) => {
          consoleLog("info", "Service call succeeded", { serviceName: name, serviceType: type, response });
          try { callback(response || {}, ""); } catch (cbErr) { console.error("[rosClient] service callback failed", cbErr); }
        },
        (err) => {
          const msg = formatError(err);
          emitError(msg);
          consoleLog("error", "Service call failed", { serviceName: name, serviceType: type, error: msg });
          try { callback(null, msg); } catch (cbErr) { console.error("[rosClient] service callback failed", cbErr); }
        }
      );
      return true;
    } catch (err) {
      const msg = formatError(err);
      emitError(msg);
      consoleLog("error", "Service call setup failed", { serviceName: name, serviceType: type, error: msg });
      try { callback(null, msg); } catch (_err) {}
      return false;
    }
  }

  function callService(serviceName, serviceType, requestObj, callback) {
    return callServiceObject(serviceName, serviceType, requestObj, (response, errorText) => {
      const responseJson = safeStringify(response || {});
      try {
        callback(responseJson, String(errorText || ""));
      } catch (cbErr) {
        console.error("[rosClient] callService callback failed", cbErr);
      }
    });
  }

  function callRosapi(serviceName, serviceType, requestObj, callback) {
    return callServiceObject(serviceName, serviceType, requestObj, callback);
  }

  function publishTopic(topicName, messageType, payloadObj) {
    consoleLog("info", "publishTopic()", { topicName, messageType, payloadObj });
    if (!state.connected || !state.ros) {
      const msg = "Cannot publish topic: not connected.";
      emitError(msg);
      consoleLog("warn", msg);
      return false;
    }
    const publisher = ensurePublisher(topicName, messageType);
    if (!publisher) {
      const msg = "Publisher could not be created. Check topic name/message type.";
      emitError(msg);
      consoleLog("error", msg, { topicName, messageType });
      return false;
    }
    try {
      const msg = new ROSLIB.Message(payloadObj && typeof payloadObj === "object" ? payloadObj : {});
      publisher.publish(msg);
      consoleLog("info", "Published topic message", { topicName, messageType, payload: payloadObj });
      return true;
    } catch (err) {
      const msg = formatError(err);
      emitError(msg);
      consoleLog("error", "publishTopic() failed", { topicName, messageType, error: msg });
      return false;
    }
  }

  function publishSpikeCmd(payloadObj) {
    const payload = {
      speed: Number(payloadObj && payloadObj.speed),
      duration: Math.max(0, Number(payloadObj && payloadObj.duration)),
    };
    return publishTopic("/spike/cmd", "std_msgs/String", { data: JSON.stringify(payload) });
  }

  function subscribeStatus(callback) {
    state.statusCallback = typeof callback === "function" ? callback : null;
    consoleLog("info", "subscribeStatus()", { hasCallback: !!state.statusCallback, connected: state.connected });
    if (state.connected) {
      attachStatusTopicIfNeeded();
    }
    return true;
  }

  function unsubscribeStatus() {
    consoleLog("info", "unsubscribeStatus()");
    state.statusCallback = null;
    unsubscribeRosTopic(state.statusTopic);
    state.statusTopic = null;
    return true;
  }

  function subscribeLogTopic(topicName, messageType, callback) {
    state.logConfig = {
      topicName: String(topicName || "").trim(),
      messageType: String(messageType || "").trim(),
    };
    state.logCallback = typeof callback === "function" ? callback : null;
    consoleLog("info", "subscribeLogTopic()", { ...state.logConfig, connected: state.connected });
    if (!state.logConfig.topicName || !state.logConfig.messageType || !state.logCallback) {
      return false;
    }
    if (state.connected) {
      attachLogTopicIfNeeded();
    }
    return true;
  }

  function unsubscribeLogTopic() {
    consoleLog("info", "unsubscribeLogTopic()");
    unsubscribeRosTopic(state.logTopic);
    state.logTopic = null;
    state.logConfig = null;
    state.logCallback = null;
    return true;
  }

  function subscribeTopic(topicName, messageType, callback) {
    const name = String(topicName || "").trim();
    const type = String(messageType || "").trim();
    if (!name || !type || typeof callback !== "function") {
      const msg = "subscribeTopic requires topicName, messageType, and callback.";
      emitError(msg);
      consoleLog("warn", msg, { topicName, messageType });
      return false;
    }
    if (!state.connected || !state.ros) {
      const msg = "Cannot subscribe topic: not connected.";
      emitError(msg);
      consoleLog("warn", msg, { topicName: name, messageType: type });
      return false;
    }

    const existing = state.topicSubscriptions.get(name);
    if (existing) {
      unsubscribeRosTopic(existing.topicObj);
      state.topicSubscriptions.delete(name);
    }

    try {
      const topicObj = new ROSLIB.Topic({
        ros: state.ros,
        name,
        messageType: type,
      });
      topicObj.subscribe((msg) => {
        try {
          callback(msg);
        } catch (cbErr) {
          console.error("[rosClient] subscribeTopic callback failed", cbErr);
        }
      });
      state.topicSubscriptions.set(name, { topicObj, messageType: type, callback });
      consoleLog("info", "Subscribed dynamic topic", { topicName: name, messageType: type });
      return true;
    } catch (err) {
      const msg = formatError(err);
      emitError(msg);
      consoleLog("error", "subscribeTopic failed", { topicName: name, messageType: type, error: msg });
      return false;
    }
  }

  function unsubscribeTopic(topicName) {
    const name = String(topicName || "").trim();
    if (!name) {
      return false;
    }
    const existing = state.topicSubscriptions.get(name);
    if (!existing) {
      return false;
    }
    unsubscribeRosTopic(existing.topicObj);
    state.topicSubscriptions.delete(name);
    consoleLog("info", "Unsubscribed dynamic topic", { topicName: name });
    return true;
  }

  function normalizeStringListResponse(response, preferredField) {
    if (response && Array.isArray(response[preferredField])) {
      return [...new Set(response[preferredField].map((x) => String(x)))].sort();
    }
    const allValues = Object.values(response || {});
    for (const value of allValues) {
      if (Array.isArray(value)) {
        return [...new Set(value.map((x) => String(x)))].sort();
      }
    }
    return [];
  }

  function listHelper(serviceName, serviceType, requestObj, fieldName, callback) {
    if (typeof callback !== "function") {
      consoleLog("warn", "listHelper() missing callback", { serviceName });
      return false;
    }
    return callRosapi(serviceName, serviceType, requestObj, (response, errorText) => {
      if (errorText) {
        try { callback("[]", errorText); } catch (_err) {}
        return;
      }
      const list = normalizeStringListResponse(response || {}, fieldName);
      try { callback(JSON.stringify(list), ""); } catch (cbErr) { console.error("[rosClient] list callback failed", cbErr); }
    });
  }

  function objectHelper(serviceName, serviceType, requestObj, callback) {
    if (typeof callback !== "function") {
      consoleLog("warn", "objectHelper() missing callback", { serviceName });
      return false;
    }
    return callRosapi(serviceName, serviceType, requestObj, (response, errorText) => {
      try {
        callback(safeStringify(response || {}), String(errorText || ""));
      } catch (cbErr) {
        console.error("[rosClient] object callback failed", cbErr);
      }
    });
  }

  function getTopics(callback) {
    return listHelper("/rosapi/topics", ROSAPI_TYPES.topics, {}, "topics", callback);
  }

  function getNodes(callback) {
    return listHelper("/rosapi/nodes", ROSAPI_TYPES.nodes, {}, "nodes", callback);
  }

  function getServices(callback) {
    return listHelper("/rosapi/services", ROSAPI_TYPES.services, {}, "services", callback);
  }

  function getTopicType(topicName, callback) {
    return objectHelper("/rosapi/topic_type", ROSAPI_TYPES.topicType, { topic: String(topicName || "") }, callback);
  }

  function getServiceType(serviceName, callback) {
    return objectHelper("/rosapi/service_type", ROSAPI_TYPES.serviceType, { service: String(serviceName || "") }, callback);
  }

  function getPublishers(topicName, callback) {
    return objectHelper("/rosapi/publishers", ROSAPI_TYPES.publishers, { topic: String(topicName || "") }, callback);
  }

  function getSubscribers(topicName, callback) {
    return objectHelper("/rosapi/subscribers", ROSAPI_TYPES.subscribers, { topic: String(topicName || "") }, callback);
  }

  function getNodeDetails(nodeName, callback) {
    return objectHelper("/rosapi/node_details", ROSAPI_TYPES.nodeDetails, { node: String(nodeName || "") }, callback);
  }

  function connect(url) {
    const targetUrl = normalizeUrl(url);
    consoleLog("info", "connect() called", { targetUrl, currentPhase: state.phase, currentUrl: state.activeUrl });

    state.manualDisconnect = false;
    state.autoFallbackUsed = false;
    state.requestedUrl = targetUrl;
    state.activeUrl = targetUrl;
    state.lastError = "";

    if (state.ros || state.activeAttempt) {
      consoleLog("warn", "Existing connection state detected, resetting before reconnect", { hasRos: !!state.ros, phase: state.phase });
      teardownTransport({ closeSocket: true });
      state.activeAttempt = null;
      state.connected = false;
    }

    return startAttempt(targetUrl);
  }

  function disconnect() {
    consoleLog("info", "disconnect() called", { phase: state.phase, activeUrl: state.activeUrl, hasRos: !!state.ros });
    state.manualDisconnect = true;
    if (!state.ros) {
      teardownTransport({ closeSocket: false });
      finalizeDisconnected("Disconnected");
      return true;
    }
    try {
      state.ros.close();
      window.setTimeout(() => {
        if (state.manualDisconnect && state.phase !== "idle" && !state.connected) {
          consoleLog("warn", "Disconnect guard forcing idle state after close timeout");
          teardownTransport({ closeSocket: false });
          finalizeDisconnected("Disconnected");
        }
      }, DISCONNECT_GUARD_MS);
      return true;
    } catch (err) {
      const msg = formatError(err);
      emitError(msg);
      consoleLog("error", "disconnect() close failed", { error: msg });
      teardownTransport({ closeSocket: false });
      finalizeDisconnected("Disconnected");
      return false;
    }
  }

  function isConnected() {
    return !!state.connected;
  }

  function setConnectionStateCallback(callback) {
    state.callbacks.connectionState = typeof callback === "function" ? callback : null;
    return true;
  }

  function setErrorCallback(callback) {
    state.callbacks.error = typeof callback === "function" ? callback : null;
    return true;
  }

  function setDebugCallback(callback) {
    state.callbacks.debug = typeof callback === "function" ? callback : null;
    return true;
  }

  function getLastError() {
    return state.lastError || "";
  }

  function getState() {
    return {
      phase: state.phase,
      connected: state.connected,
      requestedUrl: state.requestedUrl,
      activeUrl: state.activeUrl,
      lastError: state.lastError,
      autoFallbackUsed: state.autoFallbackUsed,
    };
  }

  window.rosClient = {
    connect,
    disconnect,
    isConnected,
    publishSpikeCmd,
    publishTopic,
    callService,
    subscribeStatus,
    unsubscribeStatus,
    subscribeTopic,
    unsubscribeTopic,
    subscribeLogTopic,
    unsubscribeLogTopic,
    getTopics,
    getNodes,
    getServices,
    getTopicType,
    getServiceType,
    getPublishers,
    getSubscribers,
    getNodeDetails,
    setConnectionStateCallback,
    setErrorCallback,
    setDebugCallback,
    getLastError,
    getState,
  };

  console.log("[rosClient] initialized");
})();
