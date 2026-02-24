(function () {
  const STATUS_HISTORY_MAX = 20;
  const ROS_LOG_HISTORY_MAX = 40;
  const UI_EVENT_LOG_MAX = 200;

  const state = {
    rosClient: null,
    activeTab: "explore",
    selected: null, // { kind, name }
    graph: {
      nodes: [],
      topics: [],
      services: [],
    },
    metadata: {
      topicTypeByName: new Map(),
      serviceTypeByName: new Map(),
      detailsByKey: new Map(),
    },
    statusHistory: [],
    rosLogs: [],
    uiEvents: ["(ready)"],
    lastConnected: false,
  };

  function byId(id) {
    return document.getElementById(id);
  }

  function text(id, value) {
    const el = byId(id);
    if (el) {
      el.textContent = String(value);
    }
  }

  function inputValue(id, value) {
    const el = byId(id);
    if (el) {
      el.value = String(value);
    }
  }

  function setConnDot(kind) {
    const dot = byId("conn-dot");
    if (!dot) {
      return;
    }
    dot.classList.remove("connected", "connecting", "error");
    if (kind) {
      dot.classList.add(kind);
    }
  }

  function addUiEvent(message, alsoConsoleLevel) {
    const line = `[${new Date().toLocaleTimeString()}] ${String(message)}`;
    if (state.uiEvents.length === 1 && state.uiEvents[0] === "(ready)") {
      state.uiEvents = [];
    }
    state.uiEvents.unshift(line);
    state.uiEvents = state.uiEvents.slice(0, UI_EVENT_LOG_MAX);
    renderUiEventLog();
    if (alsoConsoleLevel) {
      const fn = console[alsoConsoleLevel] || console.log;
      fn.call(console, "[UI]", message);
    }
  }

  function showError(message) {
    const banner = byId("error-banner");
    const textVal = String(message || "").trim();
    if (!banner) {
      if (textVal) {
        console.error("[UI]", textVal);
      }
      return;
    }
    if (!textVal) {
      banner.classList.add("hidden");
      banner.textContent = "(none)";
      return;
    }
    banner.classList.remove("hidden");
    banner.textContent = textVal;
    console.error("[UI]", textVal);
  }

  function clearError() {
    showError("");
  }

  function safeJsonParse(raw, fallback) {
    try {
      return JSON.parse(String(raw));
    } catch (_err) {
      return fallback;
    }
  }

  function prettyJson(value) {
    try {
      return JSON.stringify(value, null, 2);
    } catch (_err) {
      return String(value);
    }
  }

  function normalizeStringArray(value) {
    if (!Array.isArray(value)) {
      return [];
    }
    return [...new Set(value.map((x) => String(x)))].sort();
  }

  function normalizeArrayFromResponse(response, preferredField) {
    if (response && Array.isArray(response[preferredField])) {
      return normalizeStringArray(response[preferredField]);
    }
    const values = Object.values(response || {});
    for (const v of values) {
      if (Array.isArray(v)) {
        return normalizeStringArray(v);
      }
    }
    return [];
  }

  function getRosClient() {
    if (state.rosClient) {
      return state.rosClient;
    }
    const client = window.rosClient;
    if (!client) {
      throw new Error("window.rosClient is not available (ros_client.js not loaded)");
    }
    state.rosClient = client;
    return client;
  }

  function withRosClient(fn) {
    try {
      const client = getRosClient();
      return fn(client);
    } catch (err) {
      const msg = err && err.message ? err.message : String(err);
      showError(msg);
      addUiEvent(msg, "error");
      return null;
    }
  }

  function callbackToPromise(invoker) {
    return new Promise((resolve, reject) => {
      try {
        const started = invoker((jsonText, errorText) => {
          const err = String(errorText || "").trim();
          if (err) {
            reject(new Error(err));
            return;
          }
          try {
            resolve(safeJsonParse(jsonText || "{}", {}));
          } catch (parseErr) {
            reject(parseErr);
          }
        });
        if (started === false) {
          // invoker usually also triggers an error callback; keep a synchronous fallback.
          reject(new Error("Request did not start."));
        }
      } catch (err) {
        reject(err);
      }
    });
  }

  function rosCallList(methodName) {
    return withRosClient((client) =>
      callbackToPromise((done) => client[methodName](done)).then((value) =>
        Array.isArray(value) ? normalizeStringArray(value) : []
      )
    );
  }

  function rosCallObject(methodName, arg) {
    return withRosClient((client) =>
      callbackToPromise((done) => client[methodName](arg, done)).then((value) =>
        value && typeof value === "object" ? value : {}
      )
    );
  }

  function rosCallService(serviceName, serviceType, requestObj) {
    return withRosClient((client) =>
      callbackToPromise((done) => client.callService(serviceName, serviceType, requestObj || {}, done)).then((value) =>
        value && typeof value === "object" ? value : {}
      )
    );
  }

  function renderUiEventLog() {
    text("ui-log", state.uiEvents.join("\n"));
  }

  function renderRosLogs() {
    text("logs-source", state.logsSourceText || "Log source: none");
    const list = byId("logs-history");
    if (!list) {
      return;
    }
    list.innerHTML = "";
    if (!state.rosLogs.length) {
      const li = document.createElement("li");
      li.className = "muted";
      li.textContent = "No log messages yet.";
      list.appendChild(li);
      return;
    }
    for (const item of state.rosLogs) {
      const li = document.createElement("li");
      li.className = "mono";
      li.textContent = `[${item.t}] ${item.text}`;
      list.appendChild(li);
    }
  }

  function pushRosLog(textVal) {
    state.rosLogs.unshift({ t: new Date().toLocaleTimeString(), text: String(textVal || "") });
    state.rosLogs = state.rosLogs.slice(0, ROS_LOG_HISTORY_MAX);
    renderRosLogs();
  }

  function renderStatusHistory() {
    text("status-count", String(state.statusHistory.length));
    const list = byId("status-history");
    if (!list) {
      return;
    }
    list.innerHTML = "";
    if (!state.statusHistory.length) {
      const li = document.createElement("li");
      li.className = "muted";
      li.textContent = "Waiting for messages...";
      list.appendChild(li);
      return;
    }
    for (const entry of state.statusHistory) {
      const li = document.createElement("li");
      li.className = "mono";
      li.textContent = `[${entry.t}] ${entry.text}`;
      list.appendChild(li);
    }
  }

  function pushStatus(textVal) {
    const normalized = String(textVal || "");
    text("status-latest", normalized || "(empty string)");
    state.statusHistory.unshift({ t: new Date().toLocaleTimeString(), text: normalized || "(empty string)" });
    state.statusHistory = state.statusHistory.slice(0, STATUS_HISTORY_MAX);
    renderStatusHistory();
  }

  function setJsStatus(textVal) {
    text("js-status", textVal);
  }

  function renderConnectionFromPayload(payload) {
    const phase = String((payload && payload.phase) || "idle");
    const msg = String((payload && payload.message) || "");
    const url = String((payload && payload.url) || "");
    if (url) {
      const input = byId("ws-url");
      if (input) {
        input.value = url;
      }
    }

    if (phase === "connected") {
      setConnDot("connected");
      text("conn-text", msg || "Connected");
    } else if (phase === "connecting") {
      setConnDot("connecting");
      text("conn-text", msg || "Connecting...");
    } else if (phase === "error") {
      setConnDot("error");
      text("conn-text", msg || "Connection error");
    } else {
      setConnDot("");
      text("conn-text", msg || "Disconnected");
    }

    const connectedNow = !!payload.connected;
    if (connectedNow && !state.lastConnected) {
      addUiEvent("Connected. Refreshing ROS graph...", "log");
      refreshExplore().catch((err) => {
        showError(`Explore refresh failed after connect: ${err.message || err}`);
      });
    }
    state.lastConnected = connectedNow;
  }

  function setSelected(selected) {
    state.selected = selected;
    const label = selected ? `${selected.kind}: ${selected.name}` : "None";
    text("selected-summary", label);
    renderListSelections();
    renderInteractTab();
    if (selected) {
      openDetails(true);
      loadSelectedDetails().catch((err) => {
        showError(`Details load failed: ${err.message || err}`);
        addUiEvent(`Details load failed for ${selected.kind} ${selected.name}: ${err.message || err}`, "error");
      });
    } else {
      renderDetailsPanel(null);
    }
  }

  function itemKey(kind, name) {
    return `${kind}:${name}`;
  }

  function detailsCacheGet(kind, name) {
    return state.metadata.detailsByKey.get(itemKey(kind, name));
  }

  function detailsCacheSet(kind, name, value) {
    state.metadata.detailsByKey.set(itemKey(kind, name), value);
  }

  function renderList(containerId, items, kind) {
    const container = byId(containerId);
    if (!container) {
      return;
    }
    container.innerHTML = "";
    if (!items.length) {
      const div = document.createElement("div");
      div.className = "empty-list";
      div.textContent = "No items loaded.";
      container.appendChild(div);
      return;
    }
    for (const name of items) {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "list-item";
      button.dataset.kind = kind;
      button.dataset.name = name;
      button.textContent = name;
      button.addEventListener("click", () => {
        setSelected({ kind, name });
      });
      container.appendChild(button);
    }
    renderListSelections();
  }

  function renderListSelections() {
    const activeKey = state.selected ? itemKey(state.selected.kind, state.selected.name) : null;
    document.querySelectorAll(".list-item").forEach((el) => {
      const key = itemKey(el.dataset.kind, el.dataset.name);
      el.classList.toggle("active", !!activeKey && key === activeKey);
    });
  }

  function renderExploreCounts() {
    text("nodes-count", String(state.graph.nodes.length));
    text("topics-count", String(state.graph.topics.length));
    text("services-count", String(state.graph.services.length));
  }

  async function refreshExplore() {
    clearError();
    addUiEvent("Refreshing nodes/topics/services via rosapi...", "log");
    const [nodes, topics, services] = await Promise.all([
      rosCallList("getNodes"),
      rosCallList("getTopics"),
      rosCallList("getServices"),
    ]);
    state.graph.nodes = Array.isArray(nodes) ? nodes : [];
    state.graph.topics = Array.isArray(topics) ? topics : [];
    state.graph.services = Array.isArray(services) ? services : [];
    renderList("nodes-list", state.graph.nodes, "node");
    renderList("topics-list", state.graph.topics, "topic");
    renderList("services-list", state.graph.services, "service");
    renderExploreCounts();
    addUiEvent(`Explore refresh complete: ${state.graph.nodes.length} nodes, ${state.graph.topics.length} topics, ${state.graph.services.length} services`, "log");
  }

  function parseTypeField(response) {
    if (!response || typeof response !== "object") {
      return "";
    }
    if (typeof response.type === "string") {
      return response.type;
    }
    for (const [k, v] of Object.entries(response)) {
      if (typeof v === "string" && k.toLowerCase().includes("type")) {
        return v;
      }
    }
    return "";
  }

  async function fetchTopicDetails(topicName) {
    const cached = detailsCacheGet("topic", topicName);
    if (cached) {
      return cached;
    }
    const [typeResp, pubsResp, subsResp] = await Promise.all([
      rosCallObject("getTopicType", topicName),
      rosCallObject("getPublishers", topicName),
      rosCallObject("getSubscribers", topicName),
    ]);

    const topicType = parseTypeField(typeResp);
    if (topicType) {
      state.metadata.topicTypeByName.set(topicName, topicType);
    }

    const publishers = normalizeArrayFromResponse(pubsResp, "publishers");
    const subscribers = normalizeArrayFromResponse(subsResp, "subscribers");

    const details = {
      kind: "topic",
      name: topicName,
      summary: `Type ${topicType || "unknown"} | ${publishers.length} publishers | ${subscribers.length} subscribers`,
      tags: [
        `type: ${topicType || "unknown"}`,
        `publishers: ${publishers.length}`,
        `subscribers: ${subscribers.length}`,
      ],
      data: {
        topic: topicName,
        type: topicType,
        publishers,
        subscribers,
        raw: {
          topic_type: typeResp,
          publishers: pubsResp,
          subscribers: subsResp,
        },
      },
      interact: {
        kind: "topic",
        messageType: topicType || "",
      },
    };
    detailsCacheSet("topic", topicName, details);
    return details;
  }

  async function fetchServiceDetails(serviceName) {
    const cached = detailsCacheGet("service", serviceName);
    if (cached) {
      return cached;
    }
    const typeResp = await rosCallObject("getServiceType", serviceName);
    const serviceType = parseTypeField(typeResp);
    if (serviceType) {
      state.metadata.serviceTypeByName.set(serviceName, serviceType);
    }
    const details = {
      kind: "service",
      name: serviceName,
      summary: `Type ${serviceType || "unknown"}`,
      tags: [`type: ${serviceType || "unknown"}`],
      data: {
        service: serviceName,
        type: serviceType,
        raw: {
          service_type: typeResp,
        },
      },
      interact: {
        kind: "service",
        serviceType: serviceType || "",
      },
    };
    detailsCacheSet("service", serviceName, details);
    return details;
  }

  async function fetchNodeDetails(nodeName) {
    const cached = detailsCacheGet("node", nodeName);
    if (cached) {
      return cached;
    }
    const resp = await rosCallObject("getNodeDetails", nodeName);
    const publishing = normalizeArrayFromResponse({ publishing: resp.publishing || resp.publishers || [] }, "publishing");
    const subscribing = normalizeArrayFromResponse({ subscribing: resp.subscribing || resp.subscribers || [] }, "subscribing");
    const services = normalizeArrayFromResponse({ services: resp.services || [] }, "services");
    const details = {
      kind: "node",
      name: nodeName,
      summary: `${publishing.length} pubs | ${subscribing.length} subs | ${services.length} svcs`,
      tags: [`publishing: ${publishing.length}`, `subscribing: ${subscribing.length}`, `services: ${services.length}`],
      data: {
        node: nodeName,
        publishing,
        subscribing,
        services,
        raw: resp,
      },
      interact: { kind: "node" },
    };
    detailsCacheSet("node", nodeName, details);
    return details;
  }

  async function loadSelectedDetails() {
    if (!state.selected) {
      renderDetailsPanel(null);
      return;
    }
    const { kind, name } = state.selected;
    let details;
    if (kind === "topic") {
      details = await fetchTopicDetails(name);
    } else if (kind === "service") {
      details = await fetchServiceDetails(name);
    } else if (kind === "node") {
      details = await fetchNodeDetails(name);
    } else {
      details = null;
    }
    renderDetailsPanel(details);
    renderInteractTab();
  }

  function renderDetailsPanel(details) {
    if (!details) {
      text("details-kind", "Details");
      text("details-title", "Nothing selected");
      text("details-summary", "Select a node, topic, or service from Explore to inspect it.");
      text("details-content", "{}");
      const tags = byId("details-tags");
      if (tags) {
        tags.innerHTML = "";
      }
      return;
    }

    text("details-kind", details.kind);
    text("details-title", details.name);
    text("details-summary", details.summary || "");
    text("details-content", prettyJson(details.data));

    const tags = byId("details-tags");
    if (tags) {
      tags.innerHTML = "";
      for (const tag of details.tags || []) {
        const chip = document.createElement("span");
        chip.className = "detail-tag";
        chip.textContent = String(tag);
        tags.appendChild(chip);
      }
    }
  }

  function ensureTopicPayloadTemplate(topicName, messageType) {
    const textarea = byId("topic-payload-input");
    if (!textarea) {
      return;
    }
    const current = String(textarea.value || "").trim();
    const isEmpty = !current || current === "{}";
    if (!isEmpty && textarea.dataset.topicName === topicName && textarea.dataset.messageType === messageType) {
      return;
    }
    let template = {};
    if (messageType === "std_msgs/String") {
      template = { data: "hello" };
    } else if (messageType === "std_msgs/Empty") {
      template = {};
    }
    textarea.value = prettyJson(template);
    textarea.dataset.topicName = topicName;
    textarea.dataset.messageType = messageType || "";
  }

  function ensureServiceRequestTemplate(serviceName, serviceType) {
    const textarea = byId("service-request-input");
    if (!textarea) {
      return;
    }
    const current = String(textarea.value || "").trim();
    const changedSelection = textarea.dataset.serviceName !== serviceName || textarea.dataset.serviceType !== (serviceType || "");
    if (!current || current === "{}" || changedSelection) {
      textarea.value = prettyJson({});
    }
    textarea.dataset.serviceName = serviceName;
    textarea.dataset.serviceType = serviceType || "";
  }

  async function ensureTopicType(topicName) {
    if (state.metadata.topicTypeByName.has(topicName)) {
      return state.metadata.topicTypeByName.get(topicName) || "";
    }
    const resp = await rosCallObject("getTopicType", topicName);
    const t = parseTypeField(resp);
    if (t) {
      state.metadata.topicTypeByName.set(topicName, t);
    }
    return t || "";
  }

  async function ensureServiceType(serviceName) {
    if (state.metadata.serviceTypeByName.has(serviceName)) {
      return state.metadata.serviceTypeByName.get(serviceName) || "";
    }
    const resp = await rosCallObject("getServiceType", serviceName);
    const t = parseTypeField(resp);
    if (t) {
      state.metadata.serviceTypeByName.set(serviceName, t);
    }
    return t || "";
  }

  function renderInteractTab() {
    const selected = state.selected;
    const empty = byId("interact-empty");
    const topicSection = byId("interact-topic-section");
    const serviceSection = byId("interact-service-section");
    if (!empty || !topicSection || !serviceSection) {
      return;
    }

    empty.classList.add("hidden");
    topicSection.classList.add("hidden");
    serviceSection.classList.add("hidden");

    if (!selected) {
      empty.classList.remove("hidden");
      return;
    }

    if (selected.kind === "topic") {
      topicSection.classList.remove("hidden");
      inputValue("interact-topic-name", selected.name);
      const knownType = state.metadata.topicTypeByName.get(selected.name) || "";
      inputValue("interact-topic-type", knownType || "(loading topic type...)");
      ensureTopicPayloadTemplate(selected.name, knownType);
      if (!knownType) {
        ensureTopicType(selected.name)
          .then((t) => {
            inputValue("interact-topic-type", t || "(unknown)");
            ensureTopicPayloadTemplate(selected.name, t || "");
          })
          .catch((err) => {
            showError(`Topic type lookup failed: ${err.message || err}`);
            inputValue("interact-topic-type", "(lookup failed)");
          });
      }
      return;
    }

    if (selected.kind === "service") {
      serviceSection.classList.remove("hidden");
      inputValue("interact-service-name", selected.name);
      const knownType = state.metadata.serviceTypeByName.get(selected.name) || "";
      inputValue("interact-service-type", knownType || "(loading service type...)");
      ensureServiceRequestTemplate(selected.name, knownType);
      if (!knownType) {
        ensureServiceType(selected.name)
          .then((t) => {
            inputValue("interact-service-type", t || "(unknown)");
            ensureServiceRequestTemplate(selected.name, t || "");
          })
          .catch((err) => {
            showError(`Service type lookup failed: ${err.message || err}`);
            inputValue("interact-service-type", "(lookup failed)");
          });
      }
      return;
    }

    empty.classList.remove("hidden");
    empty.textContent = "Select a topic or service in Explore to load an interaction widget (nodes are inspect-only).";
  }

  function setActiveTab(tabName) {
    state.activeTab = tabName;
    document.querySelectorAll(".tab-btn").forEach((btn) => {
      btn.classList.toggle("active", btn.dataset.tab === tabName);
    });
    document.querySelectorAll(".tab-panel").forEach((panel) => {
      panel.classList.toggle("active", panel.dataset.tabPanel === tabName);
    });
    if (tabName === "interact") {
      renderInteractTab();
    }
  }

  function openDetails(forceOpen) {
    const shell = byId("app-shell");
    const btn = byId("details-toggle-btn");
    if (!shell || !btn) {
      return;
    }
    const collapsed = shell.classList.contains("details-collapsed");
    const shouldCollapse = forceOpen === true ? false : forceOpen === false ? true : !collapsed;
    shell.classList.toggle("details-collapsed", shouldCollapse);
    btn.textContent = shouldCollapse ? "Show Details" : "Hide Details";
    btn.setAttribute("aria-expanded", shouldCollapse ? "false" : "true");
  }

  function copyText(value) {
    const str = String(value || "");
    if (navigator.clipboard && navigator.clipboard.writeText) {
      return navigator.clipboard.writeText(str);
    }
    const ta = document.createElement("textarea");
    ta.value = str;
    ta.style.position = "fixed";
    ta.style.opacity = "0";
    document.body.appendChild(ta);
    ta.select();
    try {
      document.execCommand("copy");
      return Promise.resolve();
    } catch (err) {
      return Promise.reject(err);
    } finally {
      ta.remove();
    }
  }

  async function onSendTopic() {
    if (!state.selected || state.selected.kind !== "topic") {
      showError("Select a topic in Explore first.");
      return;
    }
    const topicName = state.selected.name;
    const typeField = String((byId("interact-topic-type") && byId("interact-topic-type").value) || "").trim();
    const messageType = typeField && !typeField.startsWith("(") ? typeField : await ensureTopicType(topicName);
    if (!messageType) {
      showError("Topic message type is unknown; cannot publish.");
      return;
    }

    let payload;
    try {
      payload = JSON.parse(String(byId("topic-payload-input").value || "{}"));
    } catch (err) {
      showError(`Invalid topic payload JSON: ${err.message || err}`);
      return;
    }

    const ok = withRosClient((client) => client.publishTopic(topicName, messageType, payload));
    if (ok) {
      text("interact-topic-result", `Published to ${topicName} (${messageType}) at ${new Date().toLocaleTimeString()}\n${prettyJson(payload)}`);
      addUiEvent(`Published topic ${topicName}`, "log");
      clearError();
    }
  }

  async function onCallService() {
    if (!state.selected || state.selected.kind !== "service") {
      showError("Select a service in Explore first.");
      return;
    }
    const serviceName = state.selected.name;
    const typeField = String((byId("interact-service-type") && byId("interact-service-type").value) || "").trim();
    const serviceType = typeField && !typeField.startsWith("(") ? typeField : await ensureServiceType(serviceName);
    if (!serviceType) {
      showError("Service type is unknown; cannot call service.");
      return;
    }

    let requestObj;
    try {
      requestObj = JSON.parse(String(byId("service-request-input").value || "{}"));
    } catch (err) {
      showError(`Invalid service request JSON: ${err.message || err}`);
      return;
    }

    try {
      const response = await rosCallService(serviceName, serviceType, requestObj);
      text("service-response-output", prettyJson(response));
      clearError();
      addUiEvent(`Called service ${serviceName}`, "log");
    } catch (err) {
      showError(`Service call failed: ${err.message || err}`);
      text("service-response-output", `(error) ${err.message || err}`);
    }
  }

  function attachConnectionCallbacks(client) {
    if (typeof client.setConnectionStateCallback === "function") {
      client.setConnectionStateCallback((payloadJson) => {
        const payload = safeJsonParse(payloadJson || "{}", {});
        renderConnectionFromPayload(payload);
      });
    }
    if (typeof client.setErrorCallback === "function") {
      client.setErrorCallback((message) => {
        if (String(message || "").trim()) {
          showError(String(message));
        }
      });
    }
    if (typeof client.setDebugCallback === "function") {
      client.setDebugCallback((message) => {
        addUiEvent(String(message || ""));
      });
    }
    if (typeof client.subscribeStatus === "function") {
      client.subscribeStatus((statusText) => {
        pushStatus(String(statusText || ""));
      });
    }
  }

  function attachTopBarHandlers() {
    byId("connect-btn")?.addEventListener("click", () => {
      const url = String(byId("ws-url")?.value || "ws://localhost:9090").trim() || "ws://localhost:9090";
      addUiEvent(`Connect clicked (${url})`, "log");
      clearError();
      withRosClient((client) => client.connect(url));
    });

    byId("disconnect-btn")?.addEventListener("click", () => {
      addUiEvent("Disconnect clicked", "log");
      withRosClient((client) => client.disconnect());
    });

    byId("details-toggle-btn")?.addEventListener("click", () => openDetails());
  }

  function attachTabHandlers() {
    document.querySelectorAll(".tab-btn").forEach((btn) => {
      btn.addEventListener("click", () => setActiveTab(btn.dataset.tab));
    });
  }

  function attachExploreHandlers() {
    byId("explore-refresh-btn")?.addEventListener("click", () => {
      refreshExplore().catch((err) => {
        showError(`Explore refresh failed: ${err.message || err}`);
      });
    });
  }

  function attachInteractHandlers() {
    byId("topic-send-btn")?.addEventListener("click", () => {
      onSendTopic().catch((err) => {
        showError(`Publish failed: ${err.message || err}`);
      });
    });
    byId("service-call-btn")?.addEventListener("click", () => {
      onCallService().catch((err) => {
        showError(`Service call failed: ${err.message || err}`);
      });
    });
  }

  function attachRunHandlers() {
    document.querySelectorAll(".copy-btn").forEach((btn) => {
      btn.addEventListener("click", () => {
        const value = btn.dataset.copy || "";
        copyText(value)
          .then(() => {
            text("copy-status", `Copied: ${value}`);
            addUiEvent(`Copied runbook command: ${value}`, "log");
          })
          .catch((err) => {
            showError(`Copy failed: ${err.message || err}`);
          });
      });
    });
  }

  function attachLogHandlers() {
    byId("logs-sub-rosout-btn")?.addEventListener("click", () => {
      state.rosLogs = [];
      state.logsSourceText = "Log source: /rosout (rcl_interfaces/Log)";
      renderRosLogs();
      withRosClient((client) => client.subscribeLogTopic("/rosout", "rcl_interfaces/Log", (line) => pushRosLog(line)));
      addUiEvent("Subscribed to /rosout", "log");
    });

    byId("logs-sub-ui-btn")?.addEventListener("click", () => {
      state.rosLogs = [];
      state.logsSourceText = "Log source: /ui/log (std_msgs/String)";
      renderRosLogs();
      withRosClient((client) => client.subscribeLogTopic("/ui/log", "std_msgs/String", (line) => pushRosLog(line)));
      addUiEvent("Subscribed to /ui/log", "log");
    });

    byId("logs-unsub-btn")?.addEventListener("click", () => {
      withRosClient((client) => client.unsubscribeLogTopic());
      state.logsSourceText = "Log source: none";
      renderRosLogs();
      addUiEvent("Stopped log subscription", "log");
    });
  }

  function renderInitial() {
    setJsStatus("Waiting for JS handlers...");
    setConnDot("");
    text("conn-text", "Disconnected");
    text("selected-summary", "None");
    text("status-latest", "(no messages yet)");
    renderStatusHistory();
    renderRosLogs();
    renderUiEventLog();
    renderExploreCounts();
    renderDetailsPanel(null);
    renderInteractTab();
    openDetails(false);
    renderList("nodes-list", [], "node");
    renderList("topics-list", [], "topic");
    renderList("services-list", [], "service");
  }

  function bootstrap() {
    console.log("[UI] handlers attached");
    setJsStatus("JS handlers attached");

    try {
      const client = getRosClient();
      attachConnectionCallbacks(client);
      if (typeof client.getState === "function") {
        const s = client.getState();
        renderConnectionFromPayload({
          phase: s.phase || "idle",
          connected: !!s.connected,
          url: s.activeUrl || s.requestedUrl || "",
          message: s.connected ? `Connected to ${s.activeUrl || s.requestedUrl || ""}` : "Disconnected",
        });
        if (s.lastError) {
          showError(s.lastError);
        }
      }
    } catch (err) {
      const msg = err && err.message ? err.message : String(err);
      showError(msg);
      setJsStatus("JS handlers failed (rosClient missing)");
      console.error("[UI]", msg);
      return;
    }

    attachTopBarHandlers();
    attachTabHandlers();
    attachExploreHandlers();
    attachInteractHandlers();
    attachRunHandlers();
    attachLogHandlers();

    addUiEvent("JS handlers attached", "log");
    setActiveTab("explore");
  }

  document.addEventListener("DOMContentLoaded", () => {
    renderInitial();
    bootstrap();
  });
})();
