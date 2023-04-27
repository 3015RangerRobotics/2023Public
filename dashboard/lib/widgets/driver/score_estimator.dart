import 'package:dashboard/services/dashboard_state.dart';
import 'package:flutter/material.dart';

class ScoreEstimator extends StatelessWidget {
  const ScoreEstimator({super.key});

  @override
  Widget build(BuildContext context) {
    return StreamBuilder(
        stream: DashboardState.scoringTracker(),
        builder: (context, snapshot) {
          int numLinks = 0;
          int score = 0;

          if (snapshot.hasData) {
            List<bool> scoring = snapshot.data!;

            int linkStart = -1;

            // Top row
            for (int i = 0; i < 9; i++) {
              if (scoring[i]) {
                score += 5;

                if (linkStart == -1) {
                  linkStart = i;
                } else {
                  if (i - linkStart == 2) {
                    numLinks++;
                    linkStart = -1;
                    score += 5;
                  }
                }
              } else {
                linkStart = -1;
              }
            }
            linkStart = -1;
            // Mid row
            for (int i = 9; i < 18; i++) {
              if (scoring[i]) {
                score += 3;

                if (linkStart == -1) {
                  linkStart = i;
                } else {
                  if (i - linkStart == 2) {
                    numLinks++;
                    linkStart = -1;
                    score += 5;
                  }
                }
              } else {
                linkStart = -1;
              }
            }
            linkStart = -1;
            // Bottom row
            for (int i = 18; i < 27; i++) {
              if (scoring[i]) {
                score += 2;

                if (linkStart == -1) {
                  linkStart = i;
                } else {
                  if (i - linkStart == 2) {
                    numLinks++;
                    linkStart = -1;
                    score += 5;
                  }
                }
              } else {
                linkStart = -1;
              }
            }
          }

          return Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                'Total Links: $numLinks',
                style: const TextStyle(fontSize: 24),
              ),
              Text(
                'Estimated Score: $score',
                style: const TextStyle(fontSize: 24),
              ),
            ],
          );
        });
  }
}
