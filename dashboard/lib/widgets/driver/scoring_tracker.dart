import 'package:dashboard/services/dashboard_state.dart';
import 'package:flutter/material.dart';

class ScoringTracker extends StatefulWidget {
  const ScoringTracker({super.key});

  @override
  State<ScoringTracker> createState() => _ScoringTrackerState();
}

class _ScoringTrackerState extends State<ScoringTracker> {
  List<bool> _scoring = List.generate(9 * 3, (index) => false);
  int _overrideIndex = -1;

  @override
  void initState() {
    super.initState();

    DashboardState.scoringTracker().listen((data) {
      setState(() {
        _scoring = data;
      });
    });
    DashboardState.scoringOverride().listen((data) {
      setState(() {
        _overrideIndex = data;
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    return Card(
      color: Colors.black.withOpacity(0.4),
      margin: const EdgeInsets.all(0),
      child: Column(
        mainAxisSize: MainAxisSize.max,
        children: [
          Expanded(
            flex: 1,
            child: Row(
              children: [
                for (int i = 18; i < 27; i++)
                  Expanded(
                    child: _getImageButtonForIndex(i),
                  ),
              ],
            ),
          ),
          Expanded(
            flex: 1,
            child: Row(
              children: [
                for (int i = 9; i < 18; i++)
                  Expanded(
                    child: _getImageButtonForIndex(i),
                  ),
              ],
            ),
          ),
          Expanded(
            flex: 1,
            child: Row(
              children: [
                for (int i = 0; i < 9; i++)
                  Expanded(
                    child: _getImageButtonForIndex(i),
                  ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _getImageButtonForIndex(int index) {
    return ElevatedButton(
      onLongPress: () {
        if (_overrideIndex == index) {
          setState(() {
            _overrideIndex = -1;
            DashboardState.setScoringOverride(-1);
          });
        } else {
          setState(() {
            _scoring[index] = !_scoring[index];
            DashboardState.setScoringTracker(_scoring);
          });
        }
      },
      onPressed: () {
        if (_overrideIndex == index) {
          setState(() {
            _scoring[index] = true;
            DashboardState.setScoringTracker(_scoring);
            _overrideIndex = -1;
            DashboardState.setScoringOverride(-1);
          });
        } else {
          setState(() {
            _overrideIndex = index;
            DashboardState.setScoringOverride(_overrideIndex);

            if (index < 18) {
              int col = index % 9;
              bool isCone = col != 1 && col != 4 && col != 7;

              DashboardState.setIsCubeMode(!isCone);
            }
          });
        }
      },
      style: ElevatedButton.styleFrom(
        backgroundColor: Colors.transparent,
        padding: const EdgeInsets.all(0),
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(16),
        ),
        shadowColor: Colors.transparent,
        elevation: 0,
      ),
      child: Padding(
        padding: const EdgeInsets.all(8.0),
        child: _getImageForIndex(index),
      ),
    );
  }

  Image _getImageForIndex(int index) {
    bool isOverride = _overrideIndex == index;
    bool isScored = _scoring[index];
    int col = index % 9;
    bool isCone = col != 1 && col != 4 && col != 7;
    String fileSuffix =
        isOverride ? 'override' : (isScored ? 'scored' : 'unscored');
    String pieceType = index >= 18 ? 'both' : (isCone ? 'cone' : 'cube');

    return Image.asset(
      'images/${pieceType}_$fileSuffix.png',
      fit: BoxFit.contain,
    );
  }
}
