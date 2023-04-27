import 'package:dashboard/services/dashboard_state.dart';
import 'package:flutter/material.dart';

class CubeConeButton extends StatefulWidget {
  const CubeConeButton({super.key});

  @override
  State<CubeConeButton> createState() => _CubeConeButtonState();
}

class _CubeConeButtonState extends State<CubeConeButton> {
  @override
  Widget build(BuildContext context) {
    return StreamBuilder(
        stream: DashboardState.isCubeMode(),
        builder: (context, snapshot) {
          bool cubeMode = snapshot.data ?? true;

          return ElevatedButton(
            onPressed: () {
              int override = DashboardState.getScoringOverride();
              if (override == -1 || override >= 18) {
                DashboardState.setIsCubeMode(!cubeMode);
              }
            },
            style: ElevatedButton.styleFrom(
              minimumSize: const Size(400, 400),
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(32),
              ),
              foregroundColor: Colors.white,
              backgroundColor:
                  cubeMode ? Colors.deepPurple : Colors.yellowAccent[700],
            ),
            child: Text(
              cubeMode ? 'Cube Mode' : 'Cone Mode',
              style: const TextStyle(fontSize: 48),
            ),
          );
        });
  }
}
