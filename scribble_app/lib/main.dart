import 'package:flutter/material.dart';

void main() {
  runApp(MyApp());
}

class MyApp extends StatelessWidget {
  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'ScribblePath',
      theme: ThemeData(
        // This is the theme of your application.
        //
        // Try running your application with "flutter run". You'll see the
        // application has a blue toolbar. Then, without quitting the app, try
        // changing the primarySwatch below to Colors.green and then invoke
        // "hot reload" (press "r" in the console where you ran "flutter run",
        // or simply save your changes to "hot reload" in a Flutter IDE).
        // Notice that the counter didn't reset back to zero; the application
        // is not restarted.
        primarySwatch: Colors.blue,
        primaryColor: Colors.red,
        // primaryColor: Colors.grey[900],
        // This makes the visual density adapt to the platform that you run
        // the app on. For desktop platforms, the controls will be smaller and
        // closer together (more dense) than on mobile platforms.
        visualDensity: VisualDensity.adaptivePlatformDensity,
      ),
      home: MyHomePage(title: 'Canvas'),
    );
  }
}

class MyHomePage extends StatefulWidget {
  MyHomePage({Key key, this.title}) : super(key: key);

  // This widget is the home page of your application. It is stateful, meaning
  // that it has a State object (defined below) that contains fields that affect
  // how it looks.

  // This class is the configuration for the state. It holds the values (in this
  // case the title) provided by the parent (in this case the App widget) and
  // used by the build method of the State. Fields in a Widget subclass are
  // always marked "final".

  final String title;

  @override
  _MyHomePageState createState() => _MyHomePageState();
}

class Path extends CustomPainter {
  List<Offset> points;
  Path({this.points});

  Color color = Colors.blue;
  double strokeWidth = 4;
  StrokeCap strokeType = StrokeCap.round;

  @override
  void paint(Canvas canvas, Size size) {
    Paint paint = Paint()
      ..color = color
      ..strokeWidth = strokeWidth
      ..strokeCap = strokeType;

    for (int index = 0; index < points.length - 1; index++) {
      if (points[index] != null && points[index + 1] != null) {
        canvas.drawLine(points[index], points[index + 1], paint);
      }
    }
  }

  @override
  bool shouldRepaint(Path oldDelegate) => true;
}

class _MyHomePageState extends State<MyHomePage> {
  int scale = 12; //Scale of Screen
  double startX = 0;
  double startY = 0;
  int dx = 0;
  int dy = 0;
  List data = [];
  List<Offset> points = [];

  void _onPanStartHandler(DragStartDetails details) {
    setState(() {
      this.points = [];
      points.add(details.localPosition);

      // RenderBox renderBox = context.findRenderObject();
      // points.add(renderBox.globalToLocal(details.globalPosition));
      // startX = details.globalPosition.dx;
      // startY = details.globalPosition.dy;
      // this.dx = 0;
      // this.dy = 0;
      // this.data = ["$dx:$dy"];
    });
  }

  void _onPanUpdateHandler(DragUpdateDetails details) {
    setState(() {
      points.add(details.localPosition);
      // this.dx = ((startX - details.globalPosition.dx) / scale).floor();
      // this.dy = ((startY - details.globalPosition.dy) / scale).floor();
      // this.data.add("$dx:$dy");
    });
  }

  @override
  Widget build(BuildContext context) {
    // This method is rerun every time setState is called, for instance as done
    // by the _incrementCounter method above.
    //
    // The Flutter framework has been optimized to make rerunning build methods
    // fast, so that you can just rebuild anything that needs updating rather
    // than having to individually change instances of widgets.

    return Scaffold(
        appBar: AppBar(
          // Here we take the value from the MyHomePage object that was created by
          // the App.build method, and use it to set our appbar title.
          title: Text(widget.title),
        ),
        body: GestureDetector(
            onPanStart: _onPanStartHandler,
            onPanUpdate: _onPanUpdateHandler,
            child: Container(
                color: Colors.grey[900],
                child: Stack(
                  children: <Widget>[
                    Center(
                        child: Text(
                      // "$dx, $dy\n $data",
                      "$points",
                      style: TextStyle(color: Colors.white),
                    )),
                    CustomPaint(
                        size: Size.infinite, painter: Path(points: points))
                  ],
                ))));
  }
}
