import 'package:dashboard/services/nt4.dart';
import 'package:flutter/foundation.dart';

class DashboardState {
  static const String _robotAddress = kDebugMode ? '127.0.0.1' : '10.30.15.2';

  static bool _connected = false;
  static double? _matchTime;
  static double? _lastCycleTime;
  static double? _avgCycleTime;
  static List<bool> _scoring = List.generate(9 * 3, (index) => false);
  static int _scoringOverride = -1;
  static bool _cubeMode = true;
  static bool _poleLLEnabled = true;
  static bool _pickupLLEnabled = true;

  static bool _shouldSendOnConnect = false;

  static late NT4Topic _scoringTopic;
  static late NT4Topic _scoringOverrideTopic;
  static late NT4Topic _cubeModeTopic;
  static late NT4Topic _poleLLEnabledTopic;
  static late NT4Topic _pickupLLEnabledTopic;

  static late NT4Client _client;

  static Stream<bool> connectionStatus() async* {
    yield _connected;
    bool lastYielded = _connected;

    while (true) {
      if (_connected != lastYielded) {
        yield _connected;
        lastYielded = _connected;
      }
      await Future.delayed(const Duration(seconds: 1));
    }
  }

  static Stream<double?> matchTime() async* {
    yield _matchTime;
    double? lastYielded = _matchTime;

    while (true) {
      if (_matchTime != lastYielded) {
        yield _matchTime;
        lastYielded = _matchTime;
      }
      await Future.delayed(const Duration(seconds: 1));
    }
  }

  static Stream<double?> lastCycleTime() async* {
    yield _lastCycleTime;
    double? lastYielded = _lastCycleTime;

    while (true) {
      if (_lastCycleTime != lastYielded) {
        yield _lastCycleTime;
        lastYielded = _lastCycleTime;
      }
      await Future.delayed(const Duration(seconds: 1));
    }
  }

  static Stream<double?> avgCycleTime() async* {
    yield _avgCycleTime;
    double? lastYielded = _avgCycleTime;

    while (true) {
      if (_avgCycleTime != lastYielded) {
        yield _avgCycleTime;
        lastYielded = _avgCycleTime;
      }
      await Future.delayed(const Duration(seconds: 1));
    }
  }

  static Stream<List<bool>> scoringTracker() async* {
    yield _scoring;
    List<bool> lastYielded = List.from(_scoring);

    while (true) {
      if (!listEquals(_scoring, lastYielded)) {
        yield _scoring;
        lastYielded = List.from(_scoring);
      }
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<int> scoringOverride() async* {
    yield _scoringOverride;
    int lastYielded = _scoringOverride;

    while (true) {
      if (_scoringOverride != lastYielded) {
        yield _scoringOverride;
        lastYielded = _scoringOverride;
      }
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<bool> isCubeMode() async* {
    yield _cubeMode;
    bool lastYielded = _cubeMode;

    while (true) {
      if (_cubeMode != lastYielded) {
        yield _cubeMode;
        lastYielded = _cubeMode;
      }
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<bool> poleLLEnabled() async* {
    yield _poleLLEnabled;
    bool lastYielded = _poleLLEnabled;

    while (true) {
      if (_poleLLEnabled != lastYielded) {
        yield _poleLLEnabled;
        lastYielded = _poleLLEnabled;
      }
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<bool> pickupLLEnabled() async* {
    yield _pickupLLEnabled;
    bool lastYielded = _pickupLLEnabled;

    while (true) {
      if (_pickupLLEnabled != lastYielded) {
        yield _pickupLLEnabled;
        lastYielded = _pickupLLEnabled;
      }
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static void setScoringTracker(List<bool> scoring) {
    _client.addSample(_scoringTopic, scoring);
    _scoring = scoring;
  }

  static void setScoringOverride(int index) {
    _client.addSample(_scoringOverrideTopic, index.toDouble());
    _scoringOverride = index;
  }

  static void setIsCubeMode(bool isCubeMode) {
    _client.addSample(_cubeModeTopic, isCubeMode);
    _cubeMode = isCubeMode;
  }

  static void setPoleLLEnabled(bool poleLLEnabled) {
    _client.addSample(_poleLLEnabledTopic, poleLLEnabled);
    _poleLLEnabled = poleLLEnabled;
  }

  static void setPickupLLEnabled(bool pickupLLEnabled) {
    _client.addSample(_pickupLLEnabledTopic, pickupLLEnabled);
    _pickupLLEnabled = pickupLLEnabled;
  }

  static int getScoringOverride() {
    return _scoringOverride;
  }

  static void _sendAll() {
    _client.addSample(_scoringTopic, _scoring);
    _client.addSample(_scoringOverrideTopic, _scoringOverride.toDouble());
    _client.addSample(_cubeModeTopic, _cubeMode);
    _client.addSample(_poleLLEnabledTopic, _poleLLEnabled);
    _client.addSample(_pickupLLEnabledTopic, _pickupLLEnabled);
  }

  static void init() {
    _client = NT4Client(
      serverBaseAddress: _robotAddress,
      onConnect: () {
        _connected = true;
        if (_shouldSendOnConnect) {
          Future.delayed(const Duration(milliseconds: 200), () => _sendAll());
        }
      },
      onDisconnect: () {
        _connected = false;
        _shouldSendOnConnect = true;
      },
    );

    _client
        .subscribe('/SmartDashboard/MatchTime', 1.0)
        .listen((value) => _matchTime = value as double);
    _client
        .subscribe('/SmartDashboard/CycleTracker/LastCycleTime', 1.0)
        .listen((value) => _lastCycleTime = value as double);
    _client
        .subscribe('/SmartDashboard/CycleTracker/AvgCycleTime', 1.0)
        .listen((value) => _avgCycleTime = value as double?);
    _client.subscribe('/SmartDashboard/ScoringTracker').listen((value) {
      List<bool> temp = [];
      for (Object? o in (value as List)) {
        temp.add(o as bool);
      }
      _scoring = temp;
    });
    _client
        .subscribe('/SmartDashboard/ScoringTrackerOverride')
        .listen((value) => _scoringOverride = (value as double).toInt());
    _client
        .subscribe('/SmartDashboard/CubeMode')
        .listen((value) => _cubeMode = (value as bool));
    _client
        .subscribe('/SmartDashboard/PoleLLEnabled')
        .listen((value) => _poleLLEnabled = (value as bool));
    _client
        .subscribe('/SmartDashboard/PickupLLEnabled')
        .listen((value) => _pickupLLEnabled = (value as bool));

    _scoringTopic = _client.publishNewTopic(
        '/SmartDashboard/ScoringTracker', NT4TypeStr.BOOL_ARR);
    _scoringOverrideTopic = _client.publishNewTopic(
        '/SmartDashboard/ScoringTrackerOverride', NT4TypeStr.FLOAT64);
    _cubeModeTopic =
        _client.publishNewTopic('/SmartDashboard/CubeMode', NT4TypeStr.BOOL);
    _poleLLEnabledTopic = _client.publishNewTopic(
        '/SmartDashboard/PoleLLEnabled', NT4TypeStr.BOOL);
    _pickupLLEnabledTopic = _client.publishNewTopic(
        '/SmartDashboard/PickupLLEnabled', NT4TypeStr.BOOL);

    _client.setProperties(_scoringTopic, false, true);
    _client.setProperties(_scoringOverrideTopic, false, true);
    _client.setProperties(_cubeModeTopic, false, true);
    _client.setProperties(_poleLLEnabledTopic, false, true);
    _client.setProperties(_pickupLLEnabledTopic, false, true);
  }
}
