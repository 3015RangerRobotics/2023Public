import 'dart:math';

import 'package:flutter/material.dart';
import 'package:sa3_liquid/sa3_liquid.dart';

enum Alliance {
  red,
  blue,
  none,
}

class NebulaBackground extends StatefulWidget {
  final double minSpeed;
  final double maxSpeed;
  final int density;
  final Color dotColor;
  final double particleRadius;
  final Alliance alliance;
  final bool showLightning;

  const NebulaBackground({
    this.minSpeed = 0.005,
    this.maxSpeed = 0.03,
    this.density = 1500,
    this.dotColor = const Color(0xFFAAAAAA),
    this.particleRadius = 1.5,
    this.alliance = Alliance.none,
    this.showLightning = true,
    super.key,
  });

  @override
  State<NebulaBackground> createState() => _NebulaBackgroundState();
}

class _NebulaBackgroundState extends State<NebulaBackground>
    with SingleTickerProviderStateMixin {
  late Animation<double> _animation;
  late AnimationController _controller;

  @override
  void initState() {
    super.initState();

    _controller =
        AnimationController(vsync: this, duration: const Duration(seconds: 10));
    _animation = Tween(begin: 0.0, end: 1.0).animate(_controller);
    _animation.addListener(() {
      setState(() {});
    });

    _controller.repeat();
  }

  @override
  Widget build(BuildContext context) {
    ColorScheme colorScheme = Theme.of(context).colorScheme;

    Color color1 = const Color(0x44120aff);
    Color color2 = const Color(0x44bb0206);

    if (widget.alliance == Alliance.red) {
      color1 = const Color(0x44d10017);
      color2 = const Color(0x44922f00);
    } else if (widget.alliance == Alliance.blue) {
      color1 = const Color(0x44120aff);
      color2 = const Color(0x44090582);
    }

    return Stack(
      children: [
        PlasmaRenderer(
          type: PlasmaType.infinity,
          particles: 10,
          color: color1,
          blur: 0.4,
          size: 1.01,
          speed: 1,
          offset: 0,
          blendMode: BlendMode.plus,
          particleType: ParticleType.atlas,
          variation1: 0.3,
          variation2: 0,
          variation3: 0.41,
          rotation: 0,
          child: PlasmaRenderer(
            type: PlasmaType.infinity,
            particles: 10,
            color: color2,
            blur: 0.4,
            size: 1,
            speed: 1,
            offset: 0,
            blendMode: BlendMode.plus,
            particleType: ParticleType.atlas,
            variation1: 0,
            variation2: 0,
            variation3: 0,
            rotation: 0,
            child: Container(
              color: colorScheme.surface.withOpacity(0.5),
            ),
          ),
        ),
        Positioned.fill(
          child: CustomPaint(
            painter: _ParticlesPainter(
              widget.minSpeed,
              widget.maxSpeed,
              widget.density,
              widget.dotColor,
              widget.particleRadius,
            ),
          ),
          // ),
        ),
      ],
    );
  }
}

class _ParticlesPainter extends CustomPainter {
  final double minSpeed;
  final double maxSpeed;
  final int density;
  final Color dotColor;
  final double particleRadius;

  final Random random = Random();

  static List<_Particle> particles = [];
  static Size? lastSize;

  _ParticlesPainter(
    this.minSpeed,
    this.maxSpeed,
    this.density,
    this.dotColor,
    this.particleRadius,
  );

  @override
  void paint(Canvas canvas, Size size) {
    if (particles.isEmpty) {
      // Create particles
      int numParticles = ((size.width * size.height) / density).round();
      for (int i = 0; i < numParticles; i++) {
        _Particle p = _Particle(
          position: Offset(
            (random.nextDouble() * size.width).ceilToDouble(),
            (random.nextDouble() * size.height).ceilToDouble(),
          ),
          speed: Offset(
            (random.nextDouble() * maxSpeed * 2) - maxSpeed,
            (random.nextDouble() * maxSpeed * 2) - maxSpeed,
          ),
          scale: random.nextDouble(),
          scaleSpeed: (random.nextDouble() * maxSpeed * 2) - maxSpeed,
        );

        p.speed = Offset(
          p.speed.dx.abs() < minSpeed ? minSpeed * p.speed.dx.sign : p.speed.dx,
          p.speed.dy.abs() < minSpeed ? minSpeed * p.speed.dy.sign : p.speed.dy,
        );
        p.scaleSpeed = p.scaleSpeed.abs() < minSpeed
            ? minSpeed * p.scaleSpeed.sign
            : p.scaleSpeed;

        particles.add(p);
      }
    } else if (size != lastSize) {
      // Remove particles that are outside the canvas
      for (int i = particles.length - 1; i >= 0; i--) {
        if (particles[i].position.dx > size.width ||
            particles[i].position.dy > size.height) {
          particles.removeAt(i);
        }
      }

      // Adjust particle density
      int numParticles = ((size.width * size.height) / density).round();
      if (numParticles > particles.length) {
        while (numParticles > particles.length) {
          _Particle p = _Particle(
            position: Offset(
              (random.nextDouble() * size.width).ceilToDouble(),
              (random.nextDouble() * size.height).ceilToDouble(),
            ),
            speed: Offset(
              (-maxSpeed / 2) + (random.nextDouble() * maxSpeed),
              (-maxSpeed / 2) + (random.nextDouble() * maxSpeed),
            ),
            scale: random.nextDouble(),
            scaleSpeed: (random.nextDouble() * maxSpeed * 2) - maxSpeed,
          );

          p.speed = Offset(
            p.speed.dx.abs() < minSpeed
                ? minSpeed * p.speed.dx.sign
                : p.speed.dx,
            p.speed.dy.abs() < minSpeed
                ? minSpeed * p.speed.dy.sign
                : p.speed.dy,
          );
          p.scaleSpeed = p.scaleSpeed.abs() < minSpeed
              ? minSpeed * p.scaleSpeed.sign
              : p.scaleSpeed;

          particles.add(p);
        }
      } else if (numParticles < particles.length) {
        particles.removeRange(numParticles, particles.length);
      }
    }

    // Update particle positions
    for (_Particle p in particles) {
      if (p.position.dx + p.speed.dx > size.width ||
          p.position.dx + p.speed.dx < 0) {
        p.speed = Offset(-p.speed.dx, p.speed.dy);
      }
      if (p.position.dy + p.speed.dy > size.height ||
          p.position.dy + p.speed.dy < 0) {
        p.speed = Offset(p.speed.dx, -p.speed.dy);
      }
      if (p.scale + (p.scaleSpeed / size.height) > 1 ||
          p.scale + (p.scaleSpeed / size.height) < 0.5) {
        p.scaleSpeed = -p.scaleSpeed;
      }

      p.scale += p.scaleSpeed / size.height;
      p.position += (p.speed * p.scale);
    }

    Paint dotPaint = Paint()
      ..style = PaintingStyle.fill
      ..color = dotColor;

    // Draw particles
    for (int i = 0; i < particles.length; i++) {
      _Particle p = particles[i];

      double radius = particleRadius * p.scale;
      dotPaint.color = dotColor.withOpacity(p.scale);
      canvas.drawCircle(p.position, radius, dotPaint);
    }

    lastSize = size;
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return true;
  }
}

class _Particle {
  Offset position;
  Offset speed;
  double scale;
  double scaleSpeed;

  _Particle({
    required this.position,
    required this.speed,
    required this.scale,
    required this.scaleSpeed,
  });
}
