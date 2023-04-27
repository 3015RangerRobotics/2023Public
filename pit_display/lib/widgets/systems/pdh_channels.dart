import 'dart:math';

import 'package:flutter/material.dart';
import 'package:pit_display/services/systems_state.dart';

class PDHChannels extends StatelessWidget {
  const PDHChannels({super.key});

  @override
  Widget build(BuildContext context) {
    return Card(
      child: SizedBox(
        height: 300,
        child: Padding(
          padding: const EdgeInsets.all(8.0),
          child: Column(
            children: [
              Expanded(
                child: Row(
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: [
                    Row(
                      children: [
                        _miniUpperChannel(23),
                        _miniUpperChannel(22),
                        _miniUpperChannel(21),
                        _miniUpperChannel(20),
                      ],
                    ),
                    _upperChannel(19),
                    _upperChannel(18),
                    _upperChannel(17),
                    _upperChannel(16),
                    _upperChannel(15),
                    _upperChannel(14),
                    _upperChannel(13),
                    _upperChannel(12),
                    _upperChannel(11),
                    _upperChannel(10),
                  ],
                ),
              ),
              const Divider(thickness: 0.5),
              Expanded(
                child: Row(
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: [
                    const SizedBox(width: 80),
                    _lowerChannel(0),
                    _lowerChannel(1),
                    _lowerChannel(2),
                    _lowerChannel(3),
                    _lowerChannel(4),
                    _lowerChannel(5),
                    _lowerChannel(6),
                    _lowerChannel(7),
                    _lowerChannel(8),
                    _lowerChannel(9),
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _miniUpperChannel(int channel) {
    return StreamBuilder(
      stream: SystemsState.pdhChannel(channel),
      builder: (context, snapshot) {
        int currentDraw = snapshot.data?.round() ?? 0;

        return Column(
          mainAxisAlignment: MainAxisAlignment.end,
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            const Expanded(child: SizedBox(width: 20)),
            Container(
              width: 10,
              height: min(max(1, 80 * (currentDraw / 10)), 80),
              color: Colors.green,
            ),
            const SizedBox(height: 8),
            Text('${currentDraw}A', style: const TextStyle(fontSize: 12)),
          ],
        );
      },
    );
  }

  Widget _upperChannel(int channel) {
    return StreamBuilder(
      stream: SystemsState.pdhChannel(channel),
      builder: (context, snapshot) {
        int currentDraw = snapshot.data?.round() ?? 0;

        return Column(
          mainAxisAlignment: MainAxisAlignment.end,
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            Text('Ch. $channel'),
            const Expanded(child: SizedBox(width: 50)),
            Container(
              width: 20,
              height: min(max(1, 80 * (currentDraw / 60)), 80),
              color: Colors.green,
            ),
            const SizedBox(height: 8),
            Text('${currentDraw}A'),
          ],
        );
      },
    );
  }

  Widget _lowerChannel(int channel) {
    return StreamBuilder(
      stream: SystemsState.pdhChannel(channel),
      builder: (context, snapshot) {
        int currentDraw = snapshot.data?.round() ?? 0;

        return Column(
          mainAxisAlignment: MainAxisAlignment.start,
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            Text('${currentDraw}A'),
            const SizedBox(height: 8),
            Container(
              width: 20,
              height: min(max(1, 80 * (currentDraw / 60)), 80),
              color: Colors.green,
            ),
            const Expanded(child: SizedBox(width: 50)),
            Text('Ch. $channel'),
          ],
        );
      },
    );
  }
}
