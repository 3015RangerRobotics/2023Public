import 'package:dashboard/services/dashboard_state.dart';
import 'package:flutter/material.dart';

class MatchTimer extends StatefulWidget {
  const MatchTimer({super.key});

  @override
  State<MatchTimer> createState() => _MatchTimerState();
}

class _MatchTimerState extends State<MatchTimer> {
  @override
  Widget build(BuildContext context) {
    return StreamBuilder(
      stream: DashboardState.matchTime(),
      builder: (context, snapshot) {
        String timeStr = '0:00';

        if (snapshot.hasData && snapshot.data != -1) {
          int mins = (snapshot.data! / 60).floor();
          int secs = (snapshot.data! % 60).floor();

          timeStr = '$mins:${secs.toString().padLeft(2, '0')}';
        }

        return FittedBox(
          fit: BoxFit.fitHeight,
          child: Text(
            timeStr,
            style: const TextStyle(
              fontSize: 500,
              height: 1.0,
            ),
          ),
        );
      },
    );
  }
}
