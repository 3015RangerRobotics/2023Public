import 'package:flutter/material.dart';
import 'package:pit_display/services/systems_state.dart';

class MotorTemps extends StatelessWidget {
  const MotorTemps({super.key});

  @override
  Widget build(BuildContext context) {
    return Card(
      child: SizedBox(
        height: 200,
        child: Padding(
          padding: const EdgeInsets.all(8.0),
          child: Row(
            mainAxisSize: MainAxisSize.max,
            children: [
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: [
                    _motorTemp('FL Drive', 0),
                    _motorTemp('FL Rotation', 1),
                    _motorTemp('FR Drive', 2),
                    _motorTemp('FR Rotation', 3),
                    _motorTemp('BL Drive', 4),
                  ],
                ),
              ),
              const SizedBox(width: 32),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: [
                    _motorTemp('BL Rotation', 5),
                    _motorTemp('BR Drive', 6),
                    _motorTemp('BR Rotation', 7),
                    _motorTemp('Arm Joint', 8),
                    _motorTemp('Arm Extension', 9),
                  ],
                ),
              ),
              const SizedBox(width: 32),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: [
                    _motorTemp('Turret', 10),
                    _motorTemp('Jaw', 11),
                    _motorTemp('Intake', 12),
                    // _motorTemp('Manhattan Arms', 23),
                    const Text(
                      '',
                      style: TextStyle(fontSize: 20),
                    ),
                    const Text(
                      '',
                      style: TextStyle(fontSize: 20),
                    ),
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _motorTemp(String name, int tempIdx) {
    return StreamBuilder(
      stream: SystemsState.motorTemp(tempIdx),
      builder: (context, snapshot) {
        double temp = snapshot.data ?? 0;
        Color tempColor = Colors.green;
        if (temp >= 80) {
          tempColor = Colors.red;
        } else if (temp >= 60) {
          tempColor = Colors.yellow;
        }

        return Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Text(
              name,
              style: const TextStyle(fontSize: 20),
            ),
            Text(
              '${temp.round()}°C',
              style: TextStyle(fontSize: 20, color: tempColor),
            ),
          ],
        );
      },
    );
  }
}
