import 'package:flutter/material.dart';

class SystemStatus extends StatelessWidget {
  final String label;
  final bool? systemOK;

  const SystemStatus(
    this.label, {
    super.key,
    this.systemOK,
  });

  @override
  Widget build(BuildContext context) {
    ColorScheme colorScheme = Theme.of(context).colorScheme;

    Color fillColor = Colors.transparent;
    IconData? icon;

    if (systemOK != null) {
      fillColor = systemOK! ? Colors.green : Colors.red;
      icon = systemOK! ? Icons.check : Icons.close;
    }

    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 8.0, vertical: 4.0),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        crossAxisAlignment: CrossAxisAlignment.center,
        children: [
          Container(
            width: 24,
            height: 24,
            decoration: BoxDecoration(
              borderRadius: BorderRadius.circular(12),
              color: fillColor,
              border: Border.all(
                width: 1,
                color: colorScheme.onSurface,
              ),
            ),
            child: Padding(
              padding: const EdgeInsets.only(bottom: 1.0),
              child: Icon(
                icon,
                size: 20,
              ),
            ),
          ),
          const SizedBox(width: 8),
          Text(
            label,
            style: const TextStyle(fontSize: 18),
          ),
        ],
      ),
    );
  }
}
