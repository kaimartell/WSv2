(function () {
  const STATUS_HISTORY_MAX = 20;
  const UI_EVENT_LOG_MAX = 200;
  const TOPIC_ECHO_MAX_ACTIVE = 3;
  const TOPIC_ECHO_HISTORY_MAX = 20;
  const EDU_RUN_SERVICE_NAME = "/edu/run_command";
  const EDU_RUN_SERVICE_TYPE = "spike_workshop_interfaces/srv/RunCommand";

  const TOPIC_TEMPLATES = {
    "std_msgs/String": { data: "hello" },
    "std_msgs/Empty": {},
    "geometry_msgs/Twist": {
      linear: { x: 0.0, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: 0.0 },
    },
    "std_msgs/Bool": { data: true },
    "std_msgs/Float32": { data: 0.0 },
  };

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
    uiEvents: ["(ready)"],
    filterText: "",
    servicesNodeOnly: false,
    monitor: {
      active: new Map(), // topicName -> { messageType, messages: [{t, text}] }
    },
    connection: {
      phase: "idle",
      connected: false,
      url: "",
    },
    run: {
      launchPending: false,
    },
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

  function normalizeMaybeStringArray(value) {
    if (!Array.isArray(value)) {
      return [];
    }
    const out = [];
    for (const item of value) {
      if (typeof item === "string") {
        out.push(item);
        continue;
      }
      if (Array.isArray(item) && item.length) {
        out.push(String(item[0]));
        continue;
      }
      if (item && typeof item === "object") {
        if (typeof item.topic === "string") {
          out.push(item.topic);
          continue;
        }
        if (typeof item.name === "string") {
          out.push(item.name);
          continue;
        }
      }
    }
    return normalizeStringArray(out);
  }

  function normalizeArrayFromResponse(response, preferredField) {
    if (response && Array.isArray(response[preferredField])) {
      return normalizeMaybeStringArray(response[preferredField]);
    }
    const values = Object.values(response || {});
    for (const v of values) {
      if (Array.isArray(v)) {
        return normalizeMaybeStringArray(v);
      }
    }
    return [];
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

  function setJsStatus(textVal) {
    text("js-status", textVal);
  }

  function renderUiEventLog() {
    text("ui-log", state.uiEvents.join("\n"));
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

  function getConnectionElements() {
    const toggle = byId("connect-toggle");
    const wsUrl = byId("ws-url");
    if (!toggle || !wsUrl) {
      const msg = "[UI] missing #connect-toggle or #ws-url";
      console.error(msg);
      showError(msg);
      return null;
    }
    return { toggle, wsUrl };
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
          resolve(safeJsonParse(jsonText || "{}", {}));
        });
        if (started === false) {
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

  function filterItems(items) {
    const f = state.filterText.trim().toLowerCase();
    if (!f) {
      return items;
    }
    return items.filter((name) => String(name).toLowerCase().includes(f));
  }

  function getServiceBaseList() {
    if (!state.servicesNodeOnly) {
      return state.graph.services;
    }
    if (!state.selected || state.selected.kind !== "node") {
      return [];
    }
    const details = detailsCacheGet("node", state.selected.name);
    if (!details || !details.data) {
      return [];
    }
    return normalizeMaybeStringArray(details.data.services || []);
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
      button.title = name;
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

  function renderExploreCounts(visibleNodes, visibleTopics, visibleServices, serviceBaseCount) {
    text("nodes-count", `${visibleNodes.length}/${state.graph.nodes.length}`);
    text("topics-count", `${visibleTopics.length}/${state.graph.topics.length}`);
    text("services-count", `${visibleServices.length}/${serviceBaseCount}`);
  }

  function renderGraphLists() {
    const visibleNodes = filterItems(state.graph.nodes);
    const visibleTopics = filterItems(state.graph.topics);
    const serviceBase = getServiceBaseList();
    const visibleServices = filterItems(serviceBase);

    renderList("nodes-list", visibleNodes, "node");
    renderList("topics-list", visibleTopics, "topic");
    renderList("services-list", visibleServices, "service");
    renderExploreCounts(visibleNodes, visibleTopics, visibleServices, serviceBase.length);
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

  function updateConnectionToggle() {
    const elements = getConnectionElements();
    if (!elements) {
      return;
    }
    const btn = elements.toggle;
    const urlInput = elements.wsUrl;

    const phase = state.connection.phase;
    const connected = !!state.connection.connected;

    btn.classList.remove("primary", "muted-action");

    if (phase === "connecting") {
      btn.textContent = "Connecting...";
      btn.disabled = true;
      btn.classList.add("primary");
    } else if (connected) {
      btn.textContent = "Disconnect";
      btn.disabled = false;
      btn.classList.add("muted-action");
    } else {
      btn.textContent = "Connect";
      btn.disabled = false;
      btn.classList.add("primary");
    }

    // Requirement: lock URL while connected or connecting.
    urlInput.disabled = connected || phase === "connecting";
  }

  function clearMonitorState() {
    state.monitor.active.clear();
    renderMonitorStreams();
    renderMonitorActiveCount();
  }

  function renderConnectionFromPayload(payload) {
    const phase = String((payload && payload.phase) || "idle");
    const msg = String((payload && payload.message) || "");
    const url = String((payload && payload.url) || "");
    const connectedNow = !!(payload && payload.connected);

    state.connection.phase = phase;
    state.connection.connected = connectedNow;
    state.connection.url = url;

    if (url) {
      inputValue("ws-url", url);
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

    updateConnectionToggle();

    if (connectedNow && !state.lastConnected) {
      addUiEvent("Connected. Refreshing ROS graph...", "log");
      refreshExplore().catch((err) => {
        showError(`Explore refresh failed after connect: ${err.message || err}`);
      });
    }

    if (!connectedNow && state.lastConnected) {
      clearMonitorState();
      addUiEvent("Disconnected. Cleared monitor subscriptions.", "log");
    }

    state.lastConnected = connectedNow;
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
    const publishing = normalizeMaybeStringArray(
      resp.publishing || resp.publishers || resp.publishing_topics || resp.publications || []
    );
    const subscribing = normalizeMaybeStringArray(
      resp.subscribing || resp.subscribers || resp.subscribing_topics || resp.subscriptions || []
    );
    const services = normalizeMaybeStringArray(resp.services || resp.service_names || []);
    const associatedTopics = normalizeStringArray([...publishing, ...subscribing]);

    const details = {
      kind: "node",
      name: nodeName,
      summary: `${associatedTopics.length} topics | ${services.length} services`,
      tags: [
        `topics: ${associatedTopics.length}`,
        `publishing: ${publishing.length}`,
        `subscribing: ${subscribing.length}`,
        `services: ${services.length}`,
      ],
      data: {
        node: nodeName,
        associated_topics: associatedTopics,
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
    if (state.servicesNodeOnly && kind === "node") {
      renderGraphLists();
    }
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

  function setSelected(selected) {
    state.selected = selected;
    const label = selected ? `${selected.kind}: ${selected.name}` : "None";
    text("selected-summary", label);
    renderListSelections();
    renderInteractTab();
    renderRunTopicTools();

    if (selected) {
      openDetails(true);
      loadSelectedDetails().catch((err) => {
        showError(`Details load failed: ${err.message || err}`);
        addUiEvent(`Details load failed for ${selected.kind} ${selected.name}: ${err.message || err}`, "error");
      });
      return;
    }
    renderDetailsPanel(null);
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

  function getTopicTemplate(messageType) {
    return TOPIC_TEMPLATES[messageType] || {};
  }

  function updateTopicTemplatePreview() {
    const select = byId("topic-template-select");
    const type = String((select && select.value) || "std_msgs/String");
    text("topic-template-preview", prettyJson(getTopicTemplate(type)));
  }

  function setTopicTemplateSelect(messageType) {
    const select = byId("topic-template-select");
    if (!select) {
      return;
    }
    const type = String(messageType || "").trim();
    const hasOption = Array.from(select.options).some((opt) => opt.value === type);
    select.value = hasOption ? type : "std_msgs/String";
    updateTopicTemplatePreview();
  }

  function ensureTopicPayloadTemplate(topicName, messageType) {
    const textarea = byId("topic-payload-input");
    if (!textarea) {
      return;
    }
    const current = String(textarea.value || "").trim();
    const changedSelection =
      textarea.dataset.topicName !== topicName || textarea.dataset.messageType !== (messageType || "");
    if (!current || current === "{}" || changedSelection) {
      textarea.value = prettyJson(getTopicTemplate(messageType));
    }
    textarea.dataset.topicName = topicName;
    textarea.dataset.messageType = messageType || "";
  }

  function ensureServiceRequestTemplate(serviceName, serviceType) {
    const textarea = byId("service-request-input");
    if (!textarea) {
      return;
    }
    const current = String(textarea.value || "").trim();
    const changedSelection =
      textarea.dataset.serviceName !== serviceName || textarea.dataset.serviceType !== (serviceType || "");
    if (!current || current === "{}" || changedSelection) {
      textarea.value = prettyJson({});
    }
    textarea.dataset.serviceName = serviceName;
    textarea.dataset.serviceType = serviceType || "";
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
      setTopicTemplateSelect(knownType);
      ensureTopicPayloadTemplate(selected.name, knownType);

      if (!knownType) {
        ensureTopicType(selected.name)
          .then((t) => {
            inputValue("interact-topic-type", t || "(unknown)");
            setTopicTemplateSelect(t || "");
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

  function getSelectedTopicName() {
    if (!state.selected || state.selected.kind !== "topic") {
      return "";
    }
    return String(state.selected.name || "").trim();
  }

  function renderRunTopicTools() {
    const topicTools = byId("run-topic-tools");
    const topicLabel = byId("run-selected-topic");
    if (!topicTools || !topicLabel) {
      return;
    }
    const topicName = getSelectedTopicName();
    if (!topicName) {
      topicTools.classList.add("hidden");
      topicLabel.textContent = "(none)";
      return;
    }
    topicTools.classList.remove("hidden");
    topicLabel.textContent = topicName;
  }

  function setRunLaunchIndicator(isRunning, labelText) {
    const indicator = byId("run-launch-running");
    if (!indicator) {
      return;
    }
    indicator.textContent = labelText || "Running...";
    indicator.classList.toggle("hidden", !isRunning);
    state.run.launchPending = !!isRunning;
  }

  function getRunCliPreview(commandId, argument) {
    if (commandId === "list_nodes") {
      return "ros2 node list";
    }
    if (commandId === "list_topics") {
      return "ros2 topic list";
    }
    if (commandId === "list_services") {
      return "ros2 service list";
    }
    if (commandId === "launch_ui_backend") {
      return "ros2 launch spike_workshop_ui_backend ui_backend.launch.py";
    }
    if (commandId === "launch_instrument") {
      return "ros2 launch spike_workshop_instrument instrument.launch.py";
    }
    if (commandId === "topic_info") {
      return `ros2 topic info ${argument}`;
    }
    if (commandId === "topic_hz") {
      return `ros2 topic hz ${argument}`;
    }
    if (commandId.startsWith("topic_info:")) {
      return `ros2 topic info ${commandId.split(":", 2)[1] || argument}`;
    }
    if (commandId.startsWith("topic_hz:")) {
      return `ros2 topic hz ${commandId.split(":", 2)[1] || argument}`;
    }
    return commandId;
  }

  async function runEduCommand(commandId, argument, options) {
    const opts = { isLaunch: false, ...options };
    const arg = String(argument || "").trim();
    const preview = getRunCliPreview(commandId, arg);
    text("run-cli-preview", `$ ${preview}`);
    text("run-command-output", "(running...)");

    if (opts.isLaunch) {
      setRunLaunchIndicator(true, "Running launch command...");
    }

    try {
      const response = await rosCallService(
        EDU_RUN_SERVICE_NAME,
        EDU_RUN_SERVICE_TYPE,
        {
          command_id: commandId,
          argument: arg,
        }
      );
      const success = !!(response && response.success);
      const output = String((response && response.output) || "(no output)");
      text("run-command-output", output);
      if (success) {
        clearError();
        addUiEvent(`Run command succeeded: ${commandId}`, "log");
      } else {
        showError(`Run command failed: ${commandId}`);
        addUiEvent(`Run command failed: ${commandId}`, "error");
      }
    } catch (err) {
      const msg = err && err.message ? err.message : String(err);
      text("run-command-output", `(error) ${msg}`);
      showError(`Run command error: ${msg}`);
      addUiEvent(`Run command error (${commandId}): ${msg}`, "error");
    } finally {
      if (opts.isLaunch) {
        setRunLaunchIndicator(false, "Running...");
      }
    }
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

  function formatTopicMessage(msg) {
    if (msg && typeof msg === "object" && Object.keys(msg).length === 1 && typeof msg.data === "string") {
      return msg.data;
    }
    return prettyJson(msg);
  }

  function renderMonitorActiveCount() {
    text("monitor-active-count", `${state.monitor.active.size} / ${TOPIC_ECHO_MAX_ACTIVE} active`);
  }

  function renderMonitorTopicOptions() {
    const select = byId("monitor-topic-select");
    if (!select) {
      return;
    }

    const previous = new Set(Array.from(select.selectedOptions).map((opt) => opt.value));
    select.innerHTML = "";

    for (const topicName of state.graph.topics) {
      const option = document.createElement("option");
      option.value = topicName;
      option.textContent = topicName;
      if (previous.has(topicName)) {
        option.selected = true;
      }
      select.appendChild(option);
    }
  }

  function renderMonitorStreams() {
    const root = byId("monitor-streams");
    if (!root) {
      return;
    }

    root.innerHTML = "";
    const entries = Array.from(state.monitor.active.entries()).sort((a, b) => a[0].localeCompare(b[0]));
    if (!entries.length) {
      const empty = document.createElement("div");
      empty.className = "muted";
      empty.textContent = "No active topic echo streams.";
      root.appendChild(empty);
      return;
    }

    for (const [topicName, stream] of entries) {
      const card = document.createElement("div");
      card.className = "monitor-stream-card";

      const head = document.createElement("div");
      head.className = "monitor-stream-head";

      const titleWrap = document.createElement("div");
      const title = document.createElement("div");
      title.className = "monitor-topic mono";
      title.textContent = topicName;
      const meta = document.createElement("div");
      meta.className = "monitor-meta mono";
      meta.textContent = `${stream.messageType} | ${stream.messages.length}/${TOPIC_ECHO_HISTORY_MAX}`;
      titleWrap.appendChild(title);
      titleWrap.appendChild(meta);

      const stopBtn = document.createElement("button");
      stopBtn.type = "button";
      stopBtn.textContent = "Stop";
      stopBtn.addEventListener("click", () => {
        stopEchoTopic(topicName, true);
      });

      head.appendChild(titleWrap);
      head.appendChild(stopBtn);
      card.appendChild(head);

      const box = document.createElement("div");
      box.className = "scroll-box short";
      const list = document.createElement("ul");
      list.className = "log-list";

      if (!stream.messages.length) {
        const li = document.createElement("li");
        li.className = "muted";
        li.textContent = "Waiting for messages...";
        list.appendChild(li);
      } else {
        for (const entry of stream.messages) {
          const li = document.createElement("li");
          li.className = "mono";
          li.textContent = `[${entry.t}] ${entry.text}`;
          list.appendChild(li);
        }
      }

      box.appendChild(list);
      card.appendChild(box);
      root.appendChild(card);
    }
  }

  function pushMonitorMessage(topicName, msg) {
    const stream = state.monitor.active.get(topicName);
    if (!stream) {
      return;
    }
    stream.messages.unshift({
      t: new Date().toLocaleTimeString(),
      text: formatTopicMessage(msg),
    });
    stream.messages = stream.messages.slice(0, TOPIC_ECHO_HISTORY_MAX);
    renderMonitorStreams();
  }

  async function startEchoTopic(topicName) {
    if (state.monitor.active.has(topicName)) {
      return;
    }
    if (state.monitor.active.size >= TOPIC_ECHO_MAX_ACTIVE) {
      showError(`Topic Echo limit reached (${TOPIC_ECHO_MAX_ACTIVE} active). Stop one before starting another.`);
      return;
    }

    const messageType = await ensureTopicType(topicName);
    if (!messageType) {
      showError(`Unable to determine message type for ${topicName}`);
      return;
    }

    const ok = withRosClient((client) =>
      client.subscribeTopic(topicName, messageType, (msg) => {
        pushMonitorMessage(topicName, msg);
      })
    );
    if (ok !== true) {
      return;
    }

    state.monitor.active.set(topicName, { messageType, messages: [] });
    renderMonitorActiveCount();
    renderMonitorStreams();
    addUiEvent(`Started echo for ${topicName}`, "log");
    clearError();
  }

  function stopEchoTopic(topicName, logEvent) {
    if (!state.monitor.active.has(topicName)) {
      return;
    }

    withRosClient((client) => client.unsubscribeTopic(topicName));
    state.monitor.active.delete(topicName);
    renderMonitorActiveCount();
    renderMonitorStreams();

    if (logEvent) {
      addUiEvent(`Stopped echo for ${topicName}`, "log");
    }
  }

  async function startSelectedEchoTopics() {
    const select = byId("monitor-topic-select");
    if (!select) {
      return;
    }
    const topics = Array.from(select.selectedOptions).map((opt) => opt.value);
    if (!topics.length) {
      showError("Select one or more topics to start echo.");
      return;
    }

    clearError();
    for (const topicName of topics) {
      if (state.monitor.active.size >= TOPIC_ECHO_MAX_ACTIVE && !state.monitor.active.has(topicName)) {
        showError(`Topic Echo limit reached (${TOPIC_ECHO_MAX_ACTIVE} active).`);
        break;
      }
      // eslint-disable-next-line no-await-in-loop
      await startEchoTopic(topicName);
    }
  }

  function stopSelectedEchoTopics() {
    const select = byId("monitor-topic-select");
    if (!select) {
      return;
    }
    const topics = Array.from(select.selectedOptions).map((opt) => opt.value);
    if (!topics.length) {
      showError("Select one or more topics to stop echo.");
      return;
    }

    clearError();
    for (const topicName of topics) {
      stopEchoTopic(topicName, true);
    }
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

    renderGraphLists();
    renderMonitorTopicOptions();
    addUiEvent(
      `Explore refresh complete: ${state.graph.nodes.length} nodes, ${state.graph.topics.length} topics, ${state.graph.services.length} services`,
      "log"
    );
  }

  async function onSendTopic() {
    if (!state.selected || state.selected.kind !== "topic") {
      showError("Select a topic in Explore first.");
      return;
    }

    const topicName = state.selected.name;
    const typeField = String(byId("interact-topic-type")?.value || "").trim();
    const messageType = typeField && !typeField.startsWith("(") ? typeField : await ensureTopicType(topicName);
    if (!messageType) {
      showError("Topic message type is unknown; cannot publish.");
      return;
    }

    let payload;
    try {
      payload = JSON.parse(String(byId("topic-payload-input")?.value || "{}"));
    } catch (err) {
      showError(`Invalid topic payload JSON: ${err.message || err}`);
      return;
    }

    const ok = withRosClient((client) => client.publishTopic(topicName, messageType, payload));
    if (ok === true) {
      text(
        "interact-topic-result",
        `Published to ${topicName} (${messageType}) at ${new Date().toLocaleTimeString()}\n${prettyJson(payload)}`
      );
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
    const typeField = String(byId("interact-service-type")?.value || "").trim();
    const serviceType = typeField && !typeField.startsWith("(") ? typeField : await ensureServiceType(serviceName);
    if (!serviceType) {
      showError("Service type is unknown; cannot call service.");
      return;
    }

    let requestObj;
    try {
      requestObj = JSON.parse(String(byId("service-request-input")?.value || "{}"));
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
    const elements = getConnectionElements();
    if (!elements) {
      return;
    }

    elements.toggle.addEventListener("click", () => {
      console.log("[UI] connect-toggle clicked", {
        wsUrl: String(elements.wsUrl.value || ""),
        hasRosClient: !!window.rosClient,
      });

      try {
        const phase = state.connection.phase;
        const connected = state.connection.connected;

        if (phase === "connecting") {
          return;
        }

        if (connected) {
          addUiEvent("Disconnect clicked", "log");
          withRosClient((client) => client.disconnect());
          return;
        }

        const url = String(elements.wsUrl.value || "ws://localhost:9090").trim() || "ws://localhost:9090";
        addUiEvent(`Connect clicked (${url})`, "log");
        clearError();
        withRosClient((client) => client.connect(url));
      } catch (err) {
        const msg = err && err.message ? err.message : String(err);
        console.error("[UI] connect-toggle handler exception", err);
        showError(`[UI] connect-toggle exception: ${msg}`);
      }
    });
    console.log("[UI] connect-toggle handler attached", elements.toggle);

    byId("details-toggle-btn")?.addEventListener("click", () => openDetails());
  }

  function installGlobalErrorHandlers() {
    const previousOnError = window.onerror;
    window.onerror = function onWindowError(message, source, lineno, colno, error) {
      const msg = String(message || (error && error.message) || "Unknown window error");
      console.error("[UI] window.onerror", { message, source, lineno, colno, error });
      showError(`[window.onerror] ${msg}`);
      addUiEvent(`window.onerror: ${msg}`, "error");
      if (typeof previousOnError === "function") {
        try {
          return previousOnError.apply(this, arguments);
        } catch (prevErr) {
          console.error("[UI] previous window.onerror failed", prevErr);
        }
      }
      return false;
    };

    const previousUnhandled = window.onunhandledrejection;
    window.onunhandledrejection = function onUnhandledRejection(event) {
      const reason = event && event.reason;
      const msg =
        (reason && reason.message) ||
        (typeof reason === "string" ? reason : "") ||
        "Unhandled promise rejection";
      console.error("[UI] window.onunhandledrejection", event);
      showError(`[unhandledrejection] ${msg}`);
      addUiEvent(`unhandledrejection: ${msg}`, "error");
      if (typeof previousUnhandled === "function") {
        try {
          return previousUnhandled.call(this, event);
        } catch (prevErr) {
          console.error("[UI] previous window.onunhandledrejection failed", prevErr);
        }
      }
      return false;
    };
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

    byId("explore-filter-input")?.addEventListener("input", (event) => {
      state.filterText = String(event.target.value || "");
      renderGraphLists();
    });

    byId("services-node-only-toggle")?.addEventListener("change", (event) => {
      state.servicesNodeOnly = !!event.target.checked;
      if (state.servicesNodeOnly && state.selected && state.selected.kind === "node" && !detailsCacheGet("node", state.selected.name)) {
        loadSelectedDetails()
          .catch((err) => {
            showError(`Node details lookup failed: ${err.message || err}`);
          })
          .finally(() => {
            renderGraphLists();
          });
        return;
      }
      renderGraphLists();
    });
  }

  function attachInteractHandlers() {
    byId("topic-template-select")?.addEventListener("change", () => {
      updateTopicTemplatePreview();
    });

    byId("topic-template-insert-btn")?.addEventListener("click", () => {
      const selectedType = String(byId("topic-template-select")?.value || "std_msgs/String");
      inputValue("topic-payload-input", prettyJson(getTopicTemplate(selectedType)));
      addUiEvent(`Inserted template for ${selectedType}`, "log");
    });

    byId("topic-template-copy-btn")?.addEventListener("click", () => {
      const selectedType = String(byId("topic-template-select")?.value || "std_msgs/String");
      const payloadText = prettyJson(getTopicTemplate(selectedType));
      copyText(payloadText)
        .then(() => {
          addUiEvent(`Copied template for ${selectedType}`, "log");
          clearError();
        })
        .catch((err) => {
          showError(`Copy failed: ${err.message || err}`);
        });
    });

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
    byId("run-launch-ui-btn")?.addEventListener("click", () => {
      runEduCommand("launch_ui_backend", "", { isLaunch: true }).catch((err) => {
        showError(`Launch UI Backend failed: ${err.message || err}`);
      });
    });

    byId("run-launch-instrument-btn")?.addEventListener("click", () => {
      runEduCommand("launch_instrument", "", { isLaunch: true }).catch((err) => {
        showError(`Launch Instrument failed: ${err.message || err}`);
      });
    });

    byId("run-list-nodes-btn")?.addEventListener("click", () => {
      runEduCommand("list_nodes", "").catch((err) => {
        showError(`List Nodes failed: ${err.message || err}`);
      });
    });

    byId("run-list-topics-btn")?.addEventListener("click", () => {
      runEduCommand("list_topics", "").catch((err) => {
        showError(`List Topics failed: ${err.message || err}`);
      });
    });

    byId("run-list-services-btn")?.addEventListener("click", () => {
      runEduCommand("list_services", "").catch((err) => {
        showError(`List Services failed: ${err.message || err}`);
      });
    });

    byId("run-topic-info-btn")?.addEventListener("click", () => {
      const topicName = getSelectedTopicName();
      if (!topicName) {
        showError("Select a topic in Explore to use Topic Tools.");
        return;
      }
      runEduCommand(`topic_info:${topicName}`, "").catch((err) => {
        showError(`Topic Info failed: ${err.message || err}`);
      });
    });

    byId("run-topic-hz-btn")?.addEventListener("click", () => {
      const topicName = getSelectedTopicName();
      if (!topicName) {
        showError("Select a topic in Explore to use Topic Tools.");
        return;
      }
      runEduCommand(`topic_hz:${topicName}`, "").catch((err) => {
        showError(`Topic Hz failed: ${err.message || err}`);
      });
    });

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

  function attachMonitorHandlers() {
    byId("monitor-start-btn")?.addEventListener("click", () => {
      startSelectedEchoTopics().catch((err) => {
        showError(`Start echo failed: ${err.message || err}`);
      });
    });

    byId("monitor-stop-btn")?.addEventListener("click", () => {
      stopSelectedEchoTopics();
    });
  }

  function renderInitial() {
    setJsStatus("Waiting for JS handlers...");
    setConnDot("");
    text("conn-text", "Disconnected");
    text("selected-summary", "None");
    text("status-latest", "(no messages yet)");
    inputValue("explore-filter-input", "");
    renderStatusHistory();
    renderUiEventLog();
    renderDetailsPanel(null);
    renderGraphLists();
    renderMonitorTopicOptions();
    renderMonitorActiveCount();
    renderMonitorStreams();
    renderInteractTab();
    renderRunTopicTools();
    updateTopicTemplatePreview();
    text("run-cli-preview", "(none yet)");
    text("run-command-output", "(no command run yet)");
    setRunLaunchIndicator(false, "Running...");
    updateConnectionToggle();
    openDetails(false);
  }

  function bootstrap() {
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
    attachMonitorHandlers();

    addUiEvent("JS handlers attached", "log");
    setActiveTab("explore");
  }

  document.addEventListener("DOMContentLoaded", () => {
    window.__uiHandlersAttached = true;
    console.log("[UI] handlers attached");
    installGlobalErrorHandlers();
    renderInitial();
    bootstrap();
  });
})();
