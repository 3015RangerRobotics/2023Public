import 'package:dashboard/services/dashboard_state.dart';
import 'package:flutter/material.dart';

class ConnectionStatus extends StatelessWidget {
  const ConnectionStatus({super.key});

  @override
  Widget build(BuildContext context) {
    return StreamBuilder(
        stream: DashboardState.connectionStatus(),
        builder: (context, snapshot) {
          bool connected = snapshot.data ?? false;

          return Text(
            'NT4: ${connected ? 'Connected' : 'Disconnected'}',
            style: TextStyle(
              color: connected ? Colors.green : Colors.red,
            ),
          );
        });
  }
}
