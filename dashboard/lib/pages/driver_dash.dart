import 'package:dashboard/widgets/connection_status.dart';
import 'package:dashboard/widgets/driver/cycle_tracker.dart';
import 'package:dashboard/widgets/driver/cube_cone_button.dart';
import 'package:dashboard/widgets/driver/score_estimator.dart';
import 'package:dashboard/widgets/driver/scoring_tracker.dart';
import 'package:dashboard/widgets/driver/match_timer.dart';
import 'package:dashboard/widgets/driver/vision_toggle_buttons.dart';
import 'package:flutter/material.dart';

class DriverDash extends StatelessWidget {
  const DriverDash({super.key});

  @override
  Widget build(BuildContext context) {
    return _buildLandscape();
  }

  Widget _buildLandscape() {
    return Column(
      children: [
        Expanded(
          flex: 10,
          child: Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: const [
              AspectRatio(
                aspectRatio: 9 / 3.08,
                child: Padding(
                  padding: EdgeInsets.fromLTRB(8, 8, 8, 0),
                  child: ScoringTracker(),
                ),
              ),
            ],
          ),
        ),
        Expanded(
          flex: 7,
          child: Stack(
            children: [
              const Align(
                alignment: Alignment.bottomLeft,
                child: Padding(
                  padding: EdgeInsets.symmetric(vertical: 4.0, horizontal: 8.0),
                  child: ConnectionStatus(),
                ),
              ),
              Align(
                alignment: Alignment.centerRight,
                child: Row(
                  mainAxisAlignment: MainAxisAlignment.end,
                  children: const [
                    MatchTimer(),
                    Padding(
                      padding: EdgeInsets.fromLTRB(8.0, 8.0, 0.0, 8.0),
                      child: FittedBox(
                        fit: BoxFit.fitHeight,
                        child: VisionToggleButtons(),
                      ),
                    ),
                    Padding(
                      padding: EdgeInsets.all(8.0),
                      child: FittedBox(
                        fit: BoxFit.fitHeight,
                        child: CubeConeButton(),
                      ),
                    ),
                  ],
                ),
              ),
              const Align(
                alignment: Alignment.bottomLeft,
                child: Padding(
                  padding:
                      EdgeInsets.symmetric(horizontal: 8.0, vertical: 24.0),
                  child: CycleTracker(),
                ),
              ),
              const Align(
                alignment: Alignment.topLeft,
                child: Padding(
                  padding: EdgeInsets.all(8.0),
                  child: ScoreEstimator(),
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }
}
