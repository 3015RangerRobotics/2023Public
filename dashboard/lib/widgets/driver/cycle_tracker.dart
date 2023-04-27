import 'package:dashboard/services/dashboard_state.dart';
import 'package:flutter/material.dart';

class CycleTracker extends StatelessWidget {
  const CycleTracker({super.key});

  @override
  Widget build(BuildContext context) {
    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        StreamBuilder(
            stream: DashboardState.lastCycleTime(),
            builder: (context, snapshot) {
              double time = snapshot.data ?? 0;

              return Text(
                'Last Cycle Time: ${time.toStringAsFixed(1)}s',
                style: const TextStyle(
                  fontSize: 22,
                  color: Colors.grey,
                ),
              );
            }),
        StreamBuilder(
            stream: DashboardState.avgCycleTime(),
            builder: (context, snapshot) {
              double time = snapshot.data ?? 0;

              return Text(
                'Avg. Cycle Time: ${time.toStringAsFixed(1)}s',
                style: const TextStyle(
                  fontSize: 22,
                  color: Colors.grey,
                ),
              );
            }),
      ],
    );
  }
}
