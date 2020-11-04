import 'package:flutter/material.dart';
import 'package:tuple/tuple.dart';
import 'package:http/http.dart' as http;

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
        primarySwatch: Colors.blue,
        primaryColor: Colors.red,
        visualDensity: VisualDensity.adaptivePlatformDensity,
      ),
      home: MyHomePage(title: 'Canvas'),
    );
  }
}

class MyHomePage extends StatefulWidget {
  MyHomePage({Key key, this.title}) : super(key: key);
  final String title;

  @override
  _MyHomePageState createState() => _MyHomePageState();
}

class Path extends CustomPainter {
  List<Offset> points;
  Path({this.points});

  Color color = Colors.red;
  double strokeWidth = 10;
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
  List<Tuple2<int, int>> dataPts = [];
  List<Offset> screenPts = [];

  void _onPanStartHandler(DragStartDetails details) {
    _sendLocations(dataPts);
    setState(() {
      Offset screenPt = details.localPosition;

      this.screenPts = [screenPt];
      this.dataPts = [Tuple2(0, 0)];
    });
  }

  void _onPanUpdateHandler(DragUpdateDetails details) {
    setState(() {
      Offset screenPt = details.localPosition;
      this.screenPts.add(screenPt);

      Offset startPt = screenPts[0];
      int x = ((startPt.dx - screenPt.dx) / scale).floor();
      int y = ((startPt.dy - screenPt.dy) / scale).floor();
      Tuple2<int, int> dataPt = Tuple2(x, y);

      this.dataPts.add(dataPt);
    });
  }

  void _sendLocations(dataPts) async {
    if (dataPts.length == 0) {
      return;
    }

    String url = 'http://localhost:8080/';
    print(dataPts);
    http.post(
      url,
      headers: <String, String>{
        'Content-Type': 'application/json; charset=UTF-8',
      },
      body: '$dataPts',
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(
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
                      "$dataPts",
                      style: TextStyle(color: Colors.white),
                    )),
                    CustomPaint(
                        size: Size.infinite, painter: Path(points: screenPts))
                  ],
                ))
        ),
        bottomNavigationBar: BottomAppBar(
          color: Colors.grey[700],
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: <Widget>[
              ElevatedButton(
                onPressed: null,
                //style: ButtonStyle(backgroundColor: MaterialStateProperty.all<Color>(Colors.grey[500])),
                child: Text(
                  "STOP",
                  style: TextStyle(color: Colors.white),
                ),
              ),
              ElevatedButton(
                onPressed: null,
                //style: ButtonStyle(backgroundColor: MaterialStateProperty.all<Color>(Colors.grey[500])),
                child: Text(
                  "START",
                  style: TextStyle(color: Colors.white),
                ),
              ),
              ElevatedButton(
                onPressed: null,
                //style: ButtonStyle(backgroundColor: MaterialStateProperty.all<Color>(Colors.grey[500])),
                child: Text(
                  "RESET",
                  style: TextStyle(color:Colors.white),
                ),
              )
            ],
          )
        ),
    );
  }
}

