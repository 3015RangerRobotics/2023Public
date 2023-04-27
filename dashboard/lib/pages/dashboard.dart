import 'package:dashboard/pages/driver_dash.dart';
import 'package:dashboard/widgets/nebula_background.dart';
import 'package:flutter/material.dart';

class Dashboard extends StatefulWidget {
  const Dashboard({super.key});

  @override
  State<Dashboard> createState() => _DashboardState();
}

class _DashboardState extends State<Dashboard> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Focus(
        skipTraversal: true,
        canRequestFocus: false,
        descendantsAreFocusable: false,
        descendantsAreTraversable: false,
        child: Row(
          children: [
            Expanded(
              child: Stack(
                children: const [
                  NebulaBackground(
                    alliance: Alliance.none,
                    minSpeed: 0.03,
                    maxSpeed: 0.1,
                  ),
                  DriverDash(),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
