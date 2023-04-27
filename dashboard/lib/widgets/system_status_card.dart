import 'package:flutter/material.dart';

enum Status {
  unknown('UNKNOWN'),
  ok('OK'),
  warning('WARNING'),
  error('ERROR');

  final String value;
  const Status(this.value);

  static Status fromString(String value) {
    if (value == 'OK') {
      return Status.ok;
    } else if (value == 'WARNING') {
      return Status.warning;
    } else if (value == 'ERROR') {
      return Status.error;
    } else {
      return Status.unknown;
    }
  }
}

class SystemStatusCard extends StatelessWidget {
  final String label;
  final Status status;
  final List<dynamic> faults;
  final Function(String) runCheckCallback;

  const SystemStatusCard({
    required this.label,
    required this.status,
    required this.faults,
    required this.runCheckCallback,
    super.key,
  });

  @override
  Widget build(BuildContext context) {
    Color statusColor = Colors.grey;
    if (status == Status.ok) {
      statusColor = Colors.green;
    } else if (status == Status.warning) {
      statusColor = Colors.yellow;
    } else if (status == Status.error) {
      statusColor = Colors.red;
    }

    ColorScheme colorScheme = Theme.of(context).colorScheme;

    return SizedBox(
      width: 250,
      height: 225,
      child: Card(
        child: Padding(
          padding: const EdgeInsets.all(8.0),
          child: Column(
            children: [
              Text(
                label,
                textAlign: TextAlign.center,
                style: const TextStyle(fontSize: 28),
              ),
              Row(
                children: [
                  const Text(
                    'Status:',
                    style: TextStyle(fontSize: 18),
                    textAlign: TextAlign.start,
                  ),
                  const SizedBox(width: 8),
                  Text(
                    status.value,
                    style: TextStyle(fontSize: 18, color: statusColor),
                    textAlign: TextAlign.start,
                  ),
                ],
              ),
              Row(
                children: const [
                  Text(
                    'Faults:',
                    style: TextStyle(fontSize: 18),
                    textAlign: TextAlign.start,
                  ),
                ],
              ),
              Expanded(
                child: Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 12.0),
                  child: ListView(
                    shrinkWrap: true,
                    children: [
                      if (faults.isEmpty)
                        const Text(
                          'None',
                          style: TextStyle(fontSize: 14, color: Colors.grey),
                        ),
                      for (final fault in faults)
                        Text(
                          fault.toString(),
                          style:
                              const TextStyle(fontSize: 14, color: Colors.grey),
                        ),
                    ],
                  ),
                ),
              ),
              ElevatedButton(
                onPressed: () {
                  runCheckCallback(label);
                },
                style: ElevatedButton.styleFrom(
                  foregroundColor: colorScheme.onPrimaryContainer,
                  backgroundColor: colorScheme.primaryContainer,
                ),
                child: const Text('System Check'),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
