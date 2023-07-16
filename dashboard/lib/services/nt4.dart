import 'dart:async';
import 'dart:convert';
import 'dart:math';
import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:messagepack/messagepack.dart';
import 'package:msgpack_dart/msgpack_dart.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

class NT4Client {
  final String serverBaseAddress;
  final VoidCallback? onConnect;
  final VoidCallback? onDisconnect;

  final int _startTime = DateTime.now().microsecondsSinceEpoch;

  Map<int, NT4Subscription> _subscriptions = {};
  int _subscriptionUIDCounter = 0;
  int _publishUIDCounter = 0;
  Map<String, NT4Topic> _clientPublishedTopics = {};
  Map<int, NT4Topic> _announcedTopics = {};
  late Timer _timeSyncBgEvent;
  int _clientId = 0;
  String _serverAddr = '';
  bool _serverConnectionActive = false;
  int _serverTimeOffsetUS = 0;

  WebSocketChannel? _ws;

  NT4Client({
    required this.serverBaseAddress,
    this.onConnect,
    this.onDisconnect,
  }) {
    _timeSyncBgEvent =
        Timer.periodic(const Duration(milliseconds: 5000), (timer) {
      _wsSendTimestamp();
    });

    _wsConnect();
  }

  NT4Subscription subscribe(String topic, [double period = 0.1]) {
    NT4Subscription newSub = NT4Subscription(
      topic: topic,
      uid: getNewSubUID(),
      options: NT4SubscriptionOptions(periodicRateSeconds: period),
    );

    _subscriptions[newSub.uid] = newSub;
    _wsSubscribe(newSub);
    return newSub;
  }

  NT4Subscription subscribeAllSamples(String topic) {
    NT4Subscription newSub = NT4Subscription(
      topic: topic,
      uid: getNewSubUID(),
      options: const NT4SubscriptionOptions(all: true),
    );

    _subscriptions[newSub.uid] = newSub;
    _wsSubscribe(newSub);
    return newSub;
  }

  void unSubscribe(NT4Subscription sub) {
    _subscriptions.remove(sub.uid);
    _wsUnsubscribe(sub);
  }

  void clearAllSubscriptions() {
    for (NT4Subscription sub in _subscriptions.values) {
      unSubscribe(sub);
    }
  }

  void setProperties(NT4Topic topic, bool isPersistent, bool isRetained) {
    topic.properties['persistent'] = isPersistent;
    topic.properties['retained'] = isRetained;
    _wsSetProperties(topic);
  }

  NT4Topic publishNewTopic(String name, String type) {
    NT4Topic newTopic = NT4Topic(name: name, type: type, properties: {});
    publishTopic(newTopic);
    return newTopic;
  }

  void publishTopic(NT4Topic topic) {
    topic.pubUID = getNewPubUID();
    _clientPublishedTopics[topic.name] = topic;
    _wsPublish(topic);
  }

  void unpublishTopic(NT4Topic topic) {
    _clientPublishedTopics.remove(topic.name);
    _wsUnpublish(topic);
  }

  void addSample(NT4Topic topic, dynamic data, [int? timestamp]) {
    timestamp ??= _getServerTimeUS();

    _wsSendBinary(
        serialize([topic.pubUID, timestamp, topic.getTypeId(), data]));
  }

  void addSampleFromName(String topic, dynamic data, [int? timestamp]) {
    for (NT4Topic t in _announcedTopics.values) {
      if (t.name == topic) {
        addSample(t, data, timestamp);
        return;
      }
    }
    print('[NT4] Topic not found: $topic');
  }

  int _getClientTimeUS() {
    return DateTime.now().microsecondsSinceEpoch - _startTime;
  }

  int _getServerTimeUS() {
    return _getClientTimeUS() + _serverTimeOffsetUS;
  }

  void _wsSendTimestamp() {
    var timeTopic = _announcedTopics[-1];
    if (timeTopic != null) {
      int timeToSend = _getClientTimeUS();
      addSample(timeTopic, timeToSend, 0);
    }
  }

  void _wsHandleRecieveTimestamp(int serverTimestamp, int clientTimestamp) {
    int rxTime = _getClientTimeUS();

    int rtt = rxTime - clientTimestamp;
    int serverTimeAtRx = (serverTimestamp - rtt / 2.0).round();
    _serverTimeOffsetUS = serverTimeAtRx - rxTime;
  }

  void _wsSubscribe(NT4Subscription sub) {
    _wsSendJSON('subscribe', sub._toSubscribeJson());
  }

  void _wsUnsubscribe(NT4Subscription sub) {
    _wsSendJSON('unsubscribe', sub._toUnsubscribeJson());
  }

  void _wsPublish(NT4Topic topic) {
    _wsSendJSON('publish', topic.toPublishJson());
  }

  void _wsUnpublish(NT4Topic topic) {
    _wsSendJSON('unpublish', topic.toUnpublishJson());
  }

  void _wsSetProperties(NT4Topic topic) {
    _wsSendJSON('setproperties', topic.toPropertiesJson());
  }

  void _wsSendJSON(String method, Map<String, dynamic> params) {
    _ws?.sink.add(jsonEncode([
      {
        'method': method,
        'params': params,
      }
    ]));
  }

  void _wsSendBinary(dynamic data) {
    _ws?.sink.add(data);
  }

  void _wsConnect() {
    _clientId = Random().nextInt(99999999);

    int port = 5810;
    String prefix = 'ws://';

    _serverAddr = '$prefix$serverBaseAddress:$port/nt/DartClient_$_clientId';

    _ws = WebSocketChannel.connect(Uri.parse(_serverAddr),
        protocols: ['networktables.first.wpi.edu']);

    _ws!.stream.listen(
      (data) {
        if (!_serverConnectionActive) {
          _serverConnectionActive = true;
          onConnect?.call();
        }
        _wsOnMessage(data);
      },
      onDone: _wsOnClose,
      onError: (err) {
        print('NT4 ERR: ' + err.toString());
      },
    );

    NT4Topic timeTopic = NT4Topic(
        name: "Time", type: NT4TypeStr.INT, id: -1, pubUID: -1, properties: {});
    _announcedTopics[timeTopic.id] = timeTopic;

    _wsSendTimestamp();

    for (NT4Topic topic in _clientPublishedTopics.values) {
      _wsPublish(topic);
      _wsSetProperties(topic);
    }

    for (NT4Subscription sub in _subscriptions.values) {
      _wsSubscribe(sub);
    }
  }

  void _wsOnClose() {
    _ws = null;
    _serverConnectionActive = false;

    onDisconnect?.call();

    _announcedTopics.clear();

    print('[NT4] Connection closed. Attempting to reconnect in 1s');
    Future.delayed(const Duration(seconds: 1), _wsConnect);
  }

  void _wsOnMessage(data) {
    if (data is String) {
      var rxArr = jsonDecode(data.toString());

      if (rxArr is! List) {
        print('[NT4] Ignoring text message, not an array');
      }

      for (var msg in rxArr) {
        if (msg is! Map) {
          print('[NT4] Ignoring text message, not a json object');
          continue;
        }

        var method = msg['method'];
        var params = msg['params'];

        if (method == null || method is! String) {
          print('[NT4] Ignoring text message, method not string');
          continue;
        }

        if (params == null || params is! Map) {
          print('[NT4] Ignoring text message, params not json object');
          continue;
        }

        if (method == 'announce') {
          NT4Topic? currentTopic;
          for (NT4Topic topic in _clientPublishedTopics.values) {
            if (params['name'] == topic.name) {
              currentTopic = topic;
            }
          }

          NT4Topic newTopic = NT4Topic(
              name: params['name'],
              type: params['type'],
              id: params['id'],
              pubUID: params['pubid'] ?? (currentTopic?.pubUID ?? 0),
              properties: params['properties']);
          _announcedTopics[newTopic.id] = newTopic;
        } else if (method == 'unannounce') {
          NT4Topic? removedTopic = _announcedTopics[params['id']];
          if (removedTopic == null) {
            print(
                '[NT4] Ignorining unannounce, topic was not previously announced');
            return;
          }
          _announcedTopics.remove(removedTopic.id);
        } else if (method == 'properties') {
        } else {
          print('[NT4] Ignoring text message - unknown method ' + method);
          return;
        }
      }
    } else {
      var u = Unpacker.fromList(data);

      bool done = false;
      while (!done) {
        try {
          var msg = u.unpackList();

          int topicID = msg[0] as int;
          int timestampUS = msg[1] as int;
          // int typeID = msg[2] as int;
          var value = msg[3];

          if (topicID >= 0) {
            NT4Topic topic = _announcedTopics[topicID]!;
            for (NT4Subscription sub in _subscriptions.values) {
              if (sub.topic == topic.name) {
                sub._updateValue(value);
              }
            }
          } else if (topicID == -1) {
            _wsHandleRecieveTimestamp(timestampUS, value as int);
          } else {
            print('[NT4] ignoring binary data, invalid topic ID');
          }
        } catch (err) {
          done = true;
        }
      }
    }
  }

  int getNewSubUID() {
    _subscriptionUIDCounter++;
    return _subscriptionUIDCounter + _clientId;
  }

  int getNewPubUID() {
    _publishUIDCounter++;
    return _publishUIDCounter + _clientId;
  }
}

class NT4SubscriptionOptions {
  final double periodicRateSeconds;
  final bool all;
  final bool topicsOnly;
  final bool prefix;

  const NT4SubscriptionOptions({
    this.periodicRateSeconds = 0.1,
    this.all = false,
    this.topicsOnly = false,
    this.prefix = true,
  });

  Map<String, dynamic> toJson() {
    return {
      'periodic': periodicRateSeconds,
      'all': all,
      'topicsonly': topicsOnly,
      'prefix': prefix,
    };
  }
}

class NT4Topic {
  final String name;
  final String type;
  int id;
  int pubUID;
  final Map<String, dynamic> properties;

  NT4Topic({
    required this.name,
    required this.type,
    this.id = 0,
    this.pubUID = 0,
    required this.properties,
  });

  Map<String, dynamic> toPublishJson() {
    return {
      'name': name,
      'type': type,
      'pubuid': pubUID,
    };
  }

  Map<String, dynamic> toUnpublishJson() {
    return {
      'name': name,
      'pubuid': pubUID,
    };
  }

  Map<String, dynamic> toPropertiesJson() {
    return {
      'name': name,
      'update': properties,
    };
  }

  int getTypeId() {
    return NT4TypeStr.typeMap[type]!;
  }
}

class NT4Subscription {
  final String topic;
  final NT4SubscriptionOptions options;
  final int uid;

  Object? currentValue;
  final List<Function(Object?)> _listeners = [];

  NT4Subscription({
    required this.topic,
    this.options = const NT4SubscriptionOptions(),
    this.uid = -1,
  });

  void listen(Function(Object?) onChanged) {
    _listeners.add(onChanged);
  }

  void _updateValue(Object? value) {
    currentValue = value;
    for (var listener in _listeners) {
      listener(currentValue);
    }
  }

  Map<String, dynamic> _toSubscribeJson() {
    return {
      'topics': [topic],
      'options': options.toJson(),
      'subuid': uid,
    };
  }

  Map<String, dynamic> _toUnsubscribeJson() {
    return {
      'subuid': uid,
    };
  }
}

class NT4ValueReq {
  final List<String> topics;

  const NT4ValueReq({
    this.topics = const [],
  });

  Map<String, dynamic> toGetValsJson() {
    return {
      'topics': topics,
    };
  }
}

class NT4TypeStr {
  static final Map<String, int> typeMap = {
    'boolean': 0,
    'double': 1,
    'int': 2,
    'float': 3,
    'string': 4,
    'json': 4,
    'raw': 5,
    'rpc': 5,
    'msgpack': 5,
    'protobuff': 5,
    'boolean[]': 16,
    'double[]': 17,
    'int[]': 18,
    'float[]': 19,
    'string[]': 20,
  };

  static const BOOL = 'boolean';
  static const FLOAT64 = 'double';
  static const INT = 'int';
  static const FLOAT32 = 'float';
  static const STR = 'string';
  static const JSON = 'json';
  static const BIN_RAW = 'raw';
  static const BIN_RPC = 'rpc';
  static const BIN_MSGPACK = 'msgpack';
  static const BIN_PROTOBUF = 'protobuf';
  static const BOOL_ARR = 'boolean[]';
  static const FLOAT64_ARR = 'double[]';
  static const INT_ARR = 'int[]';
  static const FLOAT32_ARR = 'float[]';
  static const STR_ARR = 'string[]';
}
