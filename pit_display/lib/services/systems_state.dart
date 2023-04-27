import 'package:pit_display/services/nt4.dart';

class SystemsState {
  static const String _robotAddress =
      '10.30.15.2'; //kDebugMode ? '127.0.0.1' : '10.30.15.2';

  static bool _connected = false;
  static late NT4Subscription _codeRuntimeSub;
  static late NT4Subscription _inputVoltageSub;
  static late NT4Subscription _canUtilSub;
  static final List<NT4Subscription> _pdhChannelSubs = [];
  static final List<NT4Subscription> _motorTempSubs = [];

  static late NT4Subscription _swerveCheckRanSub;
  static late NT4Subscription _swerveStatusSub;
  static late NT4Subscription _swerveLastFaultSub;

  static late NT4Subscription _flCheckRanSub;
  static late NT4Subscription _flStatusSub;
  static late NT4Subscription _flLastFaultSub;

  static late NT4Subscription _frCheckRanSub;
  static late NT4Subscription _frStatusSub;
  static late NT4Subscription _frLastFaultSub;

  static late NT4Subscription _blCheckRanSub;
  static late NT4Subscription _blStatusSub;
  static late NT4Subscription _blLastFaultSub;

  static late NT4Subscription _brCheckRanSub;
  static late NT4Subscription _brStatusSub;
  static late NT4Subscription _brLastFaultSub;

  static late NT4Subscription _turretCheckRanSub;
  static late NT4Subscription _turretStatusSub;
  static late NT4Subscription _turretLastFaultSub;

  static late NT4Subscription _armCheckRanSub;
  static late NT4Subscription _armStatusSub;
  static late NT4Subscription _armLastFaultSub;

  static late NT4Subscription _jawCheckRanSub;
  static late NT4Subscription _jawStatusSub;
  static late NT4Subscription _jawLastFaultSub;

  static late NT4Subscription _intakeCheckRanSub;
  static late NT4Subscription _intakeStatusSub;
  static late NT4Subscription _intakeLastFaultSub;

  static late NT4Subscription _manhattanCheckRanSub;
  static late NT4Subscription _manhattanStatusSub;
  static late NT4Subscription _manhattanLastFaultSub;

  static late NT4Topic _swerveCheckRunningTopic;
  static late NT4Topic _turretCheckRunningTopic;
  static late NT4Topic _armCheckRunningTopic;
  static late NT4Topic _jawCheckRunningTopic;
  static late NT4Topic _intakeCheckRunningTopic;
  static late NT4Topic _manhattanCheckRunningTopic;

  static late NT4Topic _allSystemsCheckRunningTopic;

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

  static Stream<List<double>> codeRuntimePlot() async* {
    List<double> values = List.generate(304, (index) => 0.0);

    await for (Object? value in _codeRuntimeSub.periodicStream()) {
      if (value != null) {
        values.add(value as double);
        values.removeAt(0);
        yield values;
      }
    }
  }

  static Stream<List<double>> inputVoltagePlot() async* {
    List<double> values = List.generate(304, (index) => 0.0);

    await for (Object? value in _inputVoltageSub.periodicStream()) {
      if (value != null) {
        values.add(value as double);
        values.removeAt(0);
        yield values;
      }
    }
  }

  static Stream<List<double>> canUtilPlot() async* {
    List<double> values = List.generate(608, (index) => 0.0);

    await for (Object? value in _canUtilSub.periodicStream()) {
      if (value != null) {
        values.add(value as double);
        values.removeAt(0);
        yield values;
      }
    }
  }

  static Stream<double> pdhChannel(int channel) async* {
    await for (Object? value in _pdhChannelSubs[channel].periodicStream()) {
      if (value != null) {
        yield value as double;
      }
    }
  }

  static Stream<double> motorTemp(int idx) async* {
    await for (Object? value in _motorTempSubs[idx].periodicStream()) {
      if (value != null) {
        yield value as double;
      }
    }
  }

  static Stream<SystemStatus> swerveStatus() async* {
    while (true) {
      bool checkRan = (_swerveCheckRanSub.currentValue ?? false) as bool;
      String status = (_swerveStatusSub.currentValue ?? '') as String;
      String lastFault = (_swerveLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> flStatus() async* {
    while (true) {
      bool checkRan = (_flCheckRanSub.currentValue ?? false) as bool;
      String status = (_flStatusSub.currentValue ?? '') as String;
      String lastFault = (_flLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> frStatus() async* {
    while (true) {
      bool checkRan = (_frCheckRanSub.currentValue ?? false) as bool;
      String status = (_frStatusSub.currentValue ?? '') as String;
      String lastFault = (_frLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> blStatus() async* {
    while (true) {
      bool checkRan = (_blCheckRanSub.currentValue ?? false) as bool;
      String status = (_blStatusSub.currentValue ?? '') as String;
      String lastFault = (_blLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> brStatus() async* {
    while (true) {
      bool checkRan = (_brCheckRanSub.currentValue ?? false) as bool;
      String status = (_brStatusSub.currentValue ?? '') as String;
      String lastFault = (_brLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> turretStatus() async* {
    while (true) {
      bool checkRan = (_turretCheckRanSub.currentValue ?? false) as bool;
      String status = (_turretStatusSub.currentValue ?? '') as String;
      String lastFault = (_turretLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> armStatus() async* {
    while (true) {
      bool checkRan = (_armCheckRanSub.currentValue ?? false) as bool;
      String status = (_armStatusSub.currentValue ?? '') as String;
      String lastFault = (_armLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> jawStatus() async* {
    while (true) {
      bool checkRan = (_jawCheckRanSub.currentValue ?? false) as bool;
      String status = (_jawStatusSub.currentValue ?? '') as String;
      String lastFault = (_jawLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> intakeStatus() async* {
    while (true) {
      bool checkRan = (_intakeCheckRanSub.currentValue ?? false) as bool;
      String status = (_intakeStatusSub.currentValue ?? '') as String;
      String lastFault = (_intakeLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> manhattanStatus() async* {
    while (true) {
      bool checkRan = (_manhattanCheckRanSub.currentValue ?? false) as bool;
      String status = (_manhattanStatusSub.currentValue ?? '') as String;
      String lastFault = (_manhattanLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static void startSwerveCheck() async {
    _client.addSample(_swerveCheckRunningTopic, false);
    await Future.delayed(const Duration(milliseconds: 200));
    _client.addSample(_swerveCheckRunningTopic, true);
  }

  static void startTurretCheck() async {
    _client.addSample(_turretCheckRunningTopic, false);
    await Future.delayed(const Duration(milliseconds: 200));
    _client.addSample(_turretCheckRunningTopic, true);
  }

  static void startArmCheck() async {
    _client.addSample(_armCheckRunningTopic, false);
    await Future.delayed(const Duration(milliseconds: 200));
    _client.addSample(_armCheckRunningTopic, true);
  }

  static void startJawCheck() async {
    _client.addSample(_jawCheckRunningTopic, false);
    await Future.delayed(const Duration(milliseconds: 200));
    _client.addSample(_jawCheckRunningTopic, true);
  }

  static void startIntakeCheck() async {
    _client.addSample(_intakeCheckRunningTopic, false);
    await Future.delayed(const Duration(milliseconds: 200));
    _client.addSample(_intakeCheckRunningTopic, true);
  }

  static void startManhattanCheck() async {
    _client.addSample(_manhattanCheckRunningTopic, false);
    await Future.delayed(const Duration(milliseconds: 200));
    _client.addSample(_manhattanCheckRunningTopic, true);
  }

  static void startAllSystemsCheck() async {
    _client.addSample(_allSystemsCheckRunningTopic, false);
    await Future.delayed(const Duration(milliseconds: 200));
    _client.addSample(_allSystemsCheckRunningTopic, true);
  }

  static void init() {
    _client = NT4Client(
      serverBaseAddress: _robotAddress,
      onConnect: () {
        _connected = true;
      },
      onDisconnect: () {
        _connected = false;
      },
    );

    _codeRuntimeSub =
        _client.subscribe('/SmartDashboard/RobotPeriodicMS', 0.033);
    _inputVoltageSub =
        _client.subscribe('/SmartDashboard/RIOInputVoltage', 0.033);
    _canUtilSub = _client.subscribe('/SmartDashboard/RIOCANUtil', 0.033);

    for (int i = 0; i <= 23; i++) {
      _pdhChannelSubs.add(_client.subscribe('/SmartDashboard/PDH/Chan$i', 0.1));
    }

    _motorTempSubs.add(
        _client.subscribe('/SmartDashboard/FLSwerveModule/DriveTemp', 1.0));
    _motorTempSubs.add(
        _client.subscribe('/SmartDashboard/FLSwerveModule/RotationTemp', 1.0));
    _motorTempSubs.add(
        _client.subscribe('/SmartDashboard/FRSwerveModule/DriveTemp', 1.0));
    _motorTempSubs.add(
        _client.subscribe('/SmartDashboard/FRSwerveModule/RotationTemp', 1.0));
    _motorTempSubs.add(
        _client.subscribe('/SmartDashboard/BLSwerveModule/DriveTemp', 1.0));
    _motorTempSubs.add(
        _client.subscribe('/SmartDashboard/BLSwerveModule/RotationTemp', 1.0));
    _motorTempSubs.add(
        _client.subscribe('/SmartDashboard/BRSwerveModule/DriveTemp', 1.0));
    _motorTempSubs.add(
        _client.subscribe('/SmartDashboard/BRSwerveModule/RotationTemp', 1.0));
    _motorTempSubs
        .add(_client.subscribe('/SmartDashboard/PinkArm/JointTemp', 1.0));
    _motorTempSubs
        .add(_client.subscribe('/SmartDashboard/PinkArm/ExtensionTemp', 1.0));
    _motorTempSubs
        .add(_client.subscribe('/SmartDashboard/Turret/TurretTemp', 1.0));
    _motorTempSubs.add(_client.subscribe('/SmartDashboard/Jaw/JawTemp', 1.0));
    _motorTempSubs
        .add(_client.subscribe('/SmartDashboard/Intake/IntakeTemp', 1.0));
    _motorTempSubs
        .add(_client.subscribe('/SmartDashboard/Manhattan/ArmTemp', 1.0));

    _swerveCheckRanSub =
        _client.subscribe('/SmartDashboard/SystemStatus/Swerve/CheckRan', 0.1);
    _swerveStatusSub =
        _client.subscribe('/SmartDashboard/SystemStatus/Swerve/Status', 0.1);
    _swerveLastFaultSub =
        _client.subscribe('/SmartDashboard/SystemStatus/Swerve/LastFault', 0.1);

    _flCheckRanSub = _client.subscribe(
        '/SmartDashboard/SystemStatus/FLSwerveModule/CheckRan', 0.1);
    _flStatusSub = _client.subscribe(
        '/SmartDashboard/SystemStatus/FLSwerveModule/Status', 0.1);
    _flLastFaultSub = _client.subscribe(
        '/SmartDashboard/SystemStatus/FLSwerveModule/LastFault', 0.1);

    _frCheckRanSub = _client.subscribe(
        '/SmartDashboard/SystemStatus/FRSwerveModule/CheckRan', 0.1);
    _frStatusSub = _client.subscribe(
        '/SmartDashboard/SystemStatus/FRSwerveModule/Status', 0.1);
    _frLastFaultSub = _client.subscribe(
        '/SmartDashboard/SystemStatus/FRSwerveModule/LastFault', 0.1);

    _blCheckRanSub = _client.subscribe(
        '/SmartDashboard/SystemStatus/BLSwerveModule/CheckRan', 0.1);
    _blStatusSub = _client.subscribe(
        '/SmartDashboard/SystemStatus/BLSwerveModule/Status', 0.1);
    _blLastFaultSub = _client.subscribe(
        '/SmartDashboard/SystemStatus/BLSwerveModule/LastFault', 0.1);

    _brCheckRanSub = _client.subscribe(
        '/SmartDashboard/SystemStatus/BRSwerveModule/CheckRan', 0.1);
    _brStatusSub = _client.subscribe(
        '/SmartDashboard/SystemStatus/BRSwerveModule/Status', 0.1);
    _brLastFaultSub = _client.subscribe(
        '/SmartDashboard/SystemStatus/BRSwerveModule/LastFault', 0.1);

    _turretCheckRanSub =
        _client.subscribe('/SmartDashboard/SystemStatus/Turret/CheckRan', 0.1);
    _turretStatusSub =
        _client.subscribe('/SmartDashboard/SystemStatus/Turret/Status', 0.1);
    _turretLastFaultSub =
        _client.subscribe('/SmartDashboard/SystemStatus/Turret/LastFault', 0.1);

    _armCheckRanSub =
        _client.subscribe('/SmartDashboard/SystemStatus/PinkArm/CheckRan', 0.1);
    _armStatusSub =
        _client.subscribe('/SmartDashboard/SystemStatus/PinkArm/Status', 0.1);
    _armLastFaultSub = _client.subscribe(
        '/SmartDashboard/SystemStatus/PinkArm/LastFault', 0.1);

    _jawCheckRanSub =
        _client.subscribe('/SmartDashboard/SystemStatus/Jaw/CheckRan', 0.1);
    _jawStatusSub =
        _client.subscribe('/SmartDashboard/SystemStatus/Jaw/Status', 0.1);
    _jawLastFaultSub =
        _client.subscribe('/SmartDashboard/SystemStatus/Jaw/LastFault', 0.1);

    _intakeCheckRanSub =
        _client.subscribe('/SmartDashboard/SystemStatus/Intake/CheckRan', 0.1);
    _intakeStatusSub =
        _client.subscribe('/SmartDashboard/SystemStatus/Intake/Status', 0.1);
    _intakeLastFaultSub =
        _client.subscribe('/SmartDashboard/SystemStatus/Intake/LastFault', 0.1);

    _manhattanCheckRanSub = _client.subscribe(
        '/SmartDashboard/SystemStatus/Manhattan/CheckRan', 0.1);
    _manhattanStatusSub =
        _client.subscribe('/SmartDashboard/SystemStatus/Manhattan/Status', 0.1);
    _manhattanLastFaultSub = _client.subscribe(
        '/SmartDashboard/SystemStatus/Manhattan/LastFault', 0.1);

    _swerveCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SystemStatus/Swerve/SystemCheck/running',
        NT4TypeStr.kBool);
    _turretCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SystemStatus/Turret/SystemCheck/running',
        NT4TypeStr.kBool);
    _armCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SystemStatus/PinkArm/SystemCheck/running',
        NT4TypeStr.kBool);
    _jawCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SystemStatus/Jaw/SystemCheck/running',
        NT4TypeStr.kBool);
    _intakeCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SystemStatus/Intake/SystemCheck/running',
        NT4TypeStr.kBool);
    _manhattanCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SystemStatus/Manhattan/SystemCheck/running',
        NT4TypeStr.kBool);
    _allSystemsCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SystemStatus/AllSystemsCheck/running',
        NT4TypeStr.kBool);
  }
}

class SystemStatus {
  final bool checkRan;
  final String status;
  final String lastFault;

  const SystemStatus({
    required this.checkRan,
    required this.status,
    required this.lastFault,
  });
}
