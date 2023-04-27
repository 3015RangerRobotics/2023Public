import 'package:flutter/material.dart';

class ConditionalWidget extends StatelessWidget {
  final Widget trueWidget;
  final Widget falseWidget;
  final bool condition;

  const ConditionalWidget(
      {required this.condition,
      required this.trueWidget,
      required this.falseWidget,
      super.key});

  @override
  Widget build(BuildContext context) {
    if (condition) {
      return trueWidget;
    } else {
      return falseWidget;
    }
  }
}
