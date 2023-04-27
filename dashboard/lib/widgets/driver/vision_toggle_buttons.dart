import 'package:dashboard/services/dashboard_state.dart';
import 'package:flutter/material.dart';

class VisionToggleButtons extends StatelessWidget {
  const VisionToggleButtons({super.key});

  @override
  Widget build(BuildContext context) {
    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        StreamBuilder(
          stream: DashboardState.poleLLEnabled(),
          builder: (context, snapshot) {
            bool poleLLEnabled = snapshot.data ?? true;

            return ElevatedButton(
              onPressed: () {
                DashboardState.setPoleLLEnabled(!poleLLEnabled);
              },
              style: ElevatedButton.styleFrom(
                fixedSize: const Size(200, 150),
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(32),
                ),
                foregroundColor: Colors.white,
                backgroundColor: poleLLEnabled ? Colors.green : null,
              ),
              child: Text(
                'Pole LL (${poleLLEnabled ? 'Enabled' : 'Disabled'})',
                style: const TextStyle(fontSize: 24),
              ),
            );
          },
        ),
        const SizedBox(height: 4),
        StreamBuilder(
          stream: DashboardState.pickupLLEnabled(),
          builder: (context, snapshot) {
            bool pickupLLEnabled = snapshot.data ?? true;

            return ElevatedButton(
              onPressed: () {
                DashboardState.setPickupLLEnabled(!pickupLLEnabled);
              },
              style: ElevatedButton.styleFrom(
                fixedSize: const Size(200, 150),
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(32),
                ),
                foregroundColor: Colors.white,
                backgroundColor: pickupLLEnabled ? Colors.green : null,
              ),
              child: Text(
                'Pickup LL (${pickupLLEnabled ? 'Enabled' : 'Disabled'})',
                style: const TextStyle(fontSize: 24),
              ),
            );
          },
        ),
      ],
    );
  }
}
