import 'package:flutter/material.dart';
import 'package:pit_display/services/systems_state.dart';
import 'package:pit_display/widgets/systems/can_util_graph.dart';
import 'package:pit_display/widgets/systems/code_performance_graph.dart';
import 'package:pit_display/widgets/systems/input_voltage_graph.dart';
import 'package:pit_display/widgets/systems/motor_temps.dart';
import 'package:pit_display/widgets/systems/pdh_channels.dart';

class SystemsPage extends StatefulWidget {
  const SystemsPage({super.key});

  @override
  State<SystemsPage> createState() => _SystemsPageState();
}

class _SystemsPageState extends State<SystemsPage> {
  final List<bool> _items = List.generate(19, (index) => false);

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: [
        Positioned(
          bottom: 0,
          right: 0,
          child: SizedBox(
            width: 500,
            height: 400,
            child: FittedBox(
              fit: BoxFit.contain,
              child: Image.asset(
                'images/highrise.png',
                filterQuality: FilterQuality.medium,
              ),
            ),
          ),
        ),
        Row(
          children: [
            Expanded(
              flex: 4,
              child: Padding(
                padding: const EdgeInsets.fromLTRB(8.0, 8.0, 0.0, 8.0),
                child: Card(
                  child: ListView(
                    children: [
                      const Text(
                        'Checklist',
                        textAlign: TextAlign.center,
                        style: TextStyle(fontSize: 42),
                      ),
                      const Divider(
                        thickness: 0.5,
                        indent: 8,
                        endIndent: 8,
                      ),
                      _checklistItem('Bumpers + Covers Off, Pins In', 15),
                      const SizedBox(height: 8),
                      const Text(
                        'Mechanical',
                        textAlign: TextAlign.center,
                        style: TextStyle(fontSize: 32),
                      ),
                      const Divider(
                        thickness: 0.5,
                        indent: 8,
                        endIndent: 8,
                      ),
                      _checklistItem('Swerve Inspection', 0),
                      Row(
                        mainAxisSize: MainAxisSize.max,
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Expanded(
                            child: Column(
                              children: [
                                _checklistItem('Turret Wiggle + Inspect', 1,
                                    Colors.redAccent),
                                _checklistItem(
                                    'Leadscrew Inspect', 3, Colors.redAccent),
                              ],
                            ),
                          ),
                          Expanded(
                            child: Column(
                              children: [
                                _checklistItem('Arm Wiggle + Inspect', 2,
                                    Colors.indigoAccent),
                                _checklistItem(
                                    'Intake Inspect', 4, Colors.indigoAccent),
                              ],
                            ),
                          ),
                        ],
                      ),
                      _checklistItem('Walk the Arm', 5),
                      Row(
                        mainAxisSize: MainAxisSize.max,
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Expanded(
                            child: Column(
                              children: [
                                _checklistItem(
                                    'Death Screw', 6, Colors.redAccent),
                              ],
                            ),
                          ),
                          Expanded(
                            child: Column(
                              children: [
                                _checklistItem(
                                    'Drag Chain', 7, Colors.indigoAccent),
                              ],
                            ),
                          ),
                        ],
                      ),
                      _checklistItem('Intake Wiggle', 8),
                      const SizedBox(height: 8),
                      const Text(
                        'Electrical',
                        textAlign: TextAlign.center,
                        style: TextStyle(fontSize: 32),
                      ),
                      const Divider(
                        thickness: 0.5,
                        indent: 8,
                        endIndent: 8,
                      ),
                      _checklistItem('Limelight Check', 9),
                      _checklistItem('Status Lights', 10),
                      _checklistItem('Battery Connection', 11),
                      _checklistItem(
                          'Turret Encoder + Underside Connections', 12),
                      _checklistItem('Arm Encoder', 13),
                      _checklistItem('RoboRIO + Ethernet Connections', 14),
                      const SizedBox(height: 8),
                      const Text(
                        'After Checks',
                        textAlign: TextAlign.center,
                        style: TextStyle(fontSize: 32),
                      ),
                      const Divider(
                        thickness: 0.5,
                        indent: 8,
                        endIndent: 8,
                      ),
                      _checklistItem('Bumpers + Covers On', 16),
                      _checklistItem('Robot On', 17),
                      _checklistItem('Systems Check', 18),
                    ],
                  ),
                ),
              ),
            ),
            Expanded(
              flex: 7,
              child: Column(
                children: [
                  Padding(
                    padding: const EdgeInsets.fromLTRB(0.0, 8.0, 8.0, 8.0),
                    child: Row(
                      // mainAxisSize: MainAxisSize.max,
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Expanded(
                          child: Column(
                            children: [
                              const MotorTemps(),
                              const PDHChannels(),
                              Row(
                                children: const [
                                  Expanded(child: CodePerformanceGraph()),
                                  Expanded(child: InputVoltageGraph()),
                                ],
                              ),
                              Row(
                                children: const [
                                  Expanded(child: CANUtilGraph()),
                                ],
                              ),
                            ],
                          ),
                        ),
                        Column(
                          children: [
                            _swerveStatusCard(),
                            _armStatusCard(),
                            _turretStatusCard(),
                            _jawStatusCard(),
                            _intakeStatusCard(),
                            _manhattanStatusCard(),
                            const SizedBox(height: 4),
                            const SizedBox(
                              width: 500,
                              child: FloatingActionButton.extended(
                                onPressed: SystemsState.startAllSystemsCheck,
                                label: Text('Run All Checks'),
                                icon: Icon(Icons.check),
                              ),
                            ),
                            StreamBuilder(
                              stream: SystemsState.connectionStatus(),
                              builder: (context, snapshot) {
                                bool connected = snapshot.data ?? false;

                                if (connected) {
                                  return const Text(
                                    'Robot Status: Connected',
                                    style: TextStyle(color: Colors.green),
                                  );
                                } else {
                                  return const Text(
                                    'Robot Status: Disconnected',
                                    style: TextStyle(color: Colors.red),
                                  );
                                }
                              },
                            ),
                          ],
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ],
    );
  }

  Widget _checklistItem(String title, int itemIndex, [Color? textColor]) {
    return CheckboxListTile(
      value: _items[itemIndex],
      controlAffinity: ListTileControlAffinity.leading,
      title: Text(
        title,
        style: TextStyle(fontSize: 20, color: textColor),
      ),
      visualDensity: VisualDensity.compact,
      onChanged: (value) {
        setState(() {
          _items[itemIndex] = value ?? false;
        });
      },
    );
  }

  Widget _armStatusCard() {
    return Card(
      child: SizedBox(
        width: 500,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            StreamBuilder(
              stream: SystemsState.armStatus(),
              builder: (context, snapshot) {
                SystemStatus status = snapshot.data ??
                    const SystemStatus(
                        checkRan: false, status: '', lastFault: '');

                Color statusColor = Colors.grey;
                IconData statusIcon = Icons.question_mark_rounded;
                if (status.checkRan) {
                  if (status.status == 'OK') {
                    statusColor = Colors.green;
                    statusIcon = Icons.check_circle_outline_rounded;
                  } else if (status.status == 'WARNING') {
                    statusColor = Colors.yellow;
                    statusIcon = Icons.warning_amber_rounded;
                  } else if (status.status == 'ERROR') {
                    statusColor = Colors.red;
                    statusIcon = Icons.error_outline_rounded;
                  }
                }

                return ListTile(
                  title: const Text(
                    'Arm',
                    style: TextStyle(fontSize: 20),
                  ),
                  leading: Icon(
                    statusIcon,
                    color: statusColor,
                  ),
                  subtitle: status.lastFault.isNotEmpty
                      ? Text(status.lastFault)
                      : null,
                  trailing: const ElevatedButton(
                    onPressed: SystemsState.startArmCheck,
                    child: Text('Run Check'),
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _turretStatusCard() {
    return Card(
      child: SizedBox(
        width: 500,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            StreamBuilder(
              stream: SystemsState.turretStatus(),
              builder: (context, snapshot) {
                SystemStatus status = snapshot.data ??
                    const SystemStatus(
                        checkRan: false, status: '', lastFault: '');

                Color statusColor = Colors.grey;
                IconData statusIcon = Icons.question_mark_rounded;
                if (status.checkRan) {
                  if (status.status == 'OK') {
                    statusColor = Colors.green;
                    statusIcon = Icons.check_circle_outline_rounded;
                  } else if (status.status == 'WARNING') {
                    statusColor = Colors.yellow;
                    statusIcon = Icons.warning_amber_rounded;
                  } else if (status.status == 'ERROR') {
                    statusColor = Colors.red;
                    statusIcon = Icons.error_outline_rounded;
                  }
                }

                return ListTile(
                  title: const Text(
                    'Turret',
                    style: TextStyle(fontSize: 20),
                  ),
                  leading: Icon(
                    statusIcon,
                    color: statusColor,
                  ),
                  subtitle: status.lastFault.isNotEmpty
                      ? Text(status.lastFault)
                      : null,
                  trailing: const ElevatedButton(
                    onPressed: SystemsState.startTurretCheck,
                    child: Text('Run Check'),
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _jawStatusCard() {
    return Card(
      child: SizedBox(
        width: 500,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            StreamBuilder(
              stream: SystemsState.jawStatus(),
              builder: (context, snapshot) {
                SystemStatus status = snapshot.data ??
                    const SystemStatus(
                        checkRan: false, status: '', lastFault: '');

                Color statusColor = Colors.grey;
                IconData statusIcon = Icons.question_mark_rounded;
                if (status.checkRan) {
                  if (status.status == 'OK') {
                    statusColor = Colors.green;
                    statusIcon = Icons.check_circle_outline_rounded;
                  } else if (status.status == 'WARNING') {
                    statusColor = Colors.yellow;
                    statusIcon = Icons.warning_amber_rounded;
                  } else if (status.status == 'ERROR') {
                    statusColor = Colors.red;
                    statusIcon = Icons.error_outline_rounded;
                  }
                }

                return ListTile(
                  title: const Text(
                    'Jaw',
                    style: TextStyle(fontSize: 20),
                  ),
                  leading: Icon(
                    statusIcon,
                    color: statusColor,
                  ),
                  subtitle: status.lastFault.isNotEmpty
                      ? Text(status.lastFault)
                      : null,
                  trailing: const ElevatedButton(
                    onPressed: SystemsState.startJawCheck,
                    child: Text('Run Check'),
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _intakeStatusCard() {
    return Card(
      child: SizedBox(
        width: 500,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            StreamBuilder(
              stream: SystemsState.intakeStatus(),
              builder: (context, snapshot) {
                SystemStatus status = snapshot.data ??
                    const SystemStatus(
                        checkRan: false, status: '', lastFault: '');

                Color statusColor = Colors.grey;
                IconData statusIcon = Icons.question_mark_rounded;
                if (status.checkRan) {
                  if (status.status == 'OK') {
                    statusColor = Colors.green;
                    statusIcon = Icons.check_circle_outline_rounded;
                  } else if (status.status == 'WARNING') {
                    statusColor = Colors.yellow;
                    statusIcon = Icons.warning_amber_rounded;
                  } else if (status.status == 'ERROR') {
                    statusColor = Colors.red;
                    statusIcon = Icons.error_outline_rounded;
                  }
                }

                return ListTile(
                  title: const Text(
                    'Intake',
                    style: TextStyle(fontSize: 20),
                  ),
                  leading: Icon(
                    statusIcon,
                    color: statusColor,
                  ),
                  subtitle: status.lastFault.isNotEmpty
                      ? Text(status.lastFault)
                      : null,
                  trailing: const ElevatedButton(
                    onPressed: SystemsState.startIntakeCheck,
                    child: Text('Run Check'),
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _swerveStatusCard() {
    return Card(
      child: SizedBox(
        width: 500,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            StreamBuilder(
              stream: SystemsState.swerveStatus(),
              builder: (context, snapshot) {
                SystemStatus status = snapshot.data ??
                    const SystemStatus(
                        checkRan: false, status: '', lastFault: '');

                Color statusColor = Colors.grey;
                IconData statusIcon = Icons.question_mark_rounded;
                if (status.checkRan) {
                  if (status.status == 'OK') {
                    statusColor = Colors.green;
                    statusIcon = Icons.check_circle_outline_rounded;
                  } else if (status.status == 'WARNING') {
                    statusColor = Colors.yellow;
                    statusIcon = Icons.warning_amber_rounded;
                  } else if (status.status == 'ERROR') {
                    statusColor = Colors.red;
                    statusIcon = Icons.error_outline_rounded;
                  }
                }

                return ListTile(
                  title: const Text(
                    'Swerve',
                    style: TextStyle(fontSize: 20),
                  ),
                  leading: Icon(
                    statusIcon,
                    color: statusColor,
                  ),
                  subtitle: status.lastFault.isNotEmpty
                      ? Text(status.lastFault)
                      : null,
                  trailing: const ElevatedButton(
                    onPressed: SystemsState.startSwerveCheck,
                    child: Text('Run Check'),
                  ),
                );
              },
            ),
            Padding(
              padding: const EdgeInsets.only(left: 32.0),
              child: StreamBuilder(
                stream: SystemsState.flStatus(),
                builder: (context, snapshot) {
                  SystemStatus status = snapshot.data ??
                      const SystemStatus(
                          checkRan: false, status: '', lastFault: '');

                  Color statusColor = Colors.grey;
                  IconData statusIcon = Icons.question_mark_rounded;
                  if (status.checkRan) {
                    if (status.status == 'OK') {
                      statusColor = Colors.green;
                      statusIcon = Icons.check_circle_outline_rounded;
                    } else if (status.status == 'WARNING') {
                      statusColor = Colors.yellow;
                      statusIcon = Icons.warning_amber_rounded;
                    } else if (status.status == 'ERROR') {
                      statusColor = Colors.red;
                      statusIcon = Icons.error_outline_rounded;
                    }
                  }

                  return ListTile(
                    title: const Text('FL Module'),
                    subtitle: status.lastFault.isNotEmpty
                        ? Text(status.lastFault)
                        : null,
                    leading: Icon(
                      statusIcon,
                      color: statusColor,
                    ),
                  );
                },
              ),
            ),
            Padding(
              padding: const EdgeInsets.only(left: 32.0),
              child: StreamBuilder(
                stream: SystemsState.frStatus(),
                builder: (context, snapshot) {
                  SystemStatus status = snapshot.data ??
                      const SystemStatus(
                          checkRan: false, status: '', lastFault: '');

                  Color statusColor = Colors.grey;
                  IconData statusIcon = Icons.question_mark_rounded;
                  if (status.checkRan) {
                    if (status.status == 'OK') {
                      statusColor = Colors.green;
                      statusIcon = Icons.check_circle_outline_rounded;
                    } else if (status.status == 'WARNING') {
                      statusColor = Colors.yellow;
                      statusIcon = Icons.warning_amber_rounded;
                    } else if (status.status == 'ERROR') {
                      statusColor = Colors.red;
                      statusIcon = Icons.error_outline_rounded;
                    }
                  }

                  return ListTile(
                    title: const Text('FR Module'),
                    subtitle: status.lastFault.isNotEmpty
                        ? Text(status.lastFault)
                        : null,
                    leading: Icon(
                      statusIcon,
                      color: statusColor,
                    ),
                  );
                },
              ),
            ),
            Padding(
              padding: const EdgeInsets.only(left: 32.0),
              child: StreamBuilder(
                stream: SystemsState.blStatus(),
                builder: (context, snapshot) {
                  SystemStatus status = snapshot.data ??
                      const SystemStatus(
                          checkRan: false, status: '', lastFault: '');

                  Color statusColor = Colors.grey;
                  IconData statusIcon = Icons.question_mark_rounded;
                  if (status.checkRan) {
                    if (status.status == 'OK') {
                      statusColor = Colors.green;
                      statusIcon = Icons.check_circle_outline_rounded;
                    } else if (status.status == 'WARNING') {
                      statusColor = Colors.yellow;
                      statusIcon = Icons.warning_amber_rounded;
                    } else if (status.status == 'ERROR') {
                      statusColor = Colors.red;
                      statusIcon = Icons.error_outline_rounded;
                    }
                  }

                  return ListTile(
                    title: const Text('BL Module'),
                    subtitle: status.lastFault.isNotEmpty
                        ? Text(status.lastFault)
                        : null,
                    leading: Icon(
                      statusIcon,
                      color: statusColor,
                    ),
                  );
                },
              ),
            ),
            Padding(
              padding: const EdgeInsets.only(left: 32.0),
              child: StreamBuilder(
                stream: SystemsState.brStatus(),
                builder: (context, snapshot) {
                  SystemStatus status = snapshot.data ??
                      const SystemStatus(
                          checkRan: false, status: '', lastFault: '');

                  Color statusColor = Colors.grey;
                  IconData statusIcon = Icons.question_mark_rounded;
                  if (status.checkRan) {
                    if (status.status == 'OK') {
                      statusColor = Colors.green;
                      statusIcon = Icons.check_circle_outline_rounded;
                    } else if (status.status == 'WARNING') {
                      statusColor = Colors.yellow;
                      statusIcon = Icons.warning_amber_rounded;
                    } else if (status.status == 'ERROR') {
                      statusColor = Colors.red;
                      statusIcon = Icons.error_outline_rounded;
                    }
                  }

                  return ListTile(
                    title: const Text('BR Module'),
                    subtitle: status.lastFault.isNotEmpty
                        ? Text(status.lastFault)
                        : null,
                    leading: Icon(
                      statusIcon,
                      color: statusColor,
                    ),
                  );
                },
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _manhattanStatusCard() {
    return Card(
      child: SizedBox(
        width: 500,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            StreamBuilder(
                stream: SystemsState.manhattanStatus(),
                builder: (context, snapshot) {
                  SystemStatus status = snapshot.data ??
                      const SystemStatus(
                          checkRan: false, status: '', lastFault: '');

                  Color statusColor = Colors.grey;
                  IconData statusIcon = Icons.question_mark_rounded;
                  if (status.checkRan) {
                    if (status.status == 'OK') {
                      statusColor = Colors.green;
                      statusIcon = Icons.check_circle_outline_rounded;
                    } else if (status.status == 'WARNING') {
                      statusColor = Colors.yellow;
                      statusIcon = Icons.warning_amber_rounded;
                    } else if (status.status == 'ERROR') {
                      statusColor = Colors.red;
                      statusIcon = Icons.error_outline_rounded;
                    }
                  }

                  return ListTile(
                    title: const Text(
                      'Manhattan',
                      style: TextStyle(fontSize: 20),
                    ),
                    leading: Icon(
                      statusIcon,
                      color: statusColor,
                    ),
                    subtitle: status.lastFault.isNotEmpty
                        ? Text(status.lastFault)
                        : null,
                    trailing: const ElevatedButton(
                      onPressed: SystemsState.startManhattanCheck,
                      child: Text('Run Check'),
                    ),
                  );
                }),
          ],
        ),
      ),
    );
  }
}
