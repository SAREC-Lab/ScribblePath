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
      home: MyHomePage(title: 'ScribblePath'),
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

String url = 'http://172.16.1.146:8080';

class _MyHomePageState extends State<MyHomePage> {
  int scale = 50; //Scale of Screen
  List<Tuple2<int, int>> dataPts = [];
  List<Offset> screenPts = [];

  void _onPanStartHandler(DragStartDetails details) {
    //_sendLocations(dataPts);
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

      if (dataPts.isEmpty) {
        this.dataPts.add(dataPt);
      } else if (dataPt != dataPts[dataPts.length - 1]) {
        this.dataPts.add(dataPt);
      }
    });
  }

  void _reset() {
    setState(() {
      print("called setState() in _reset()");
      this.screenPts = [];
      this.dataPts = [Tuple2(0, 0)];
    });
  }

  void _sendLocations(dataPts) async {
    if (dataPts.length == 0) {
      return;
    }

    //String url = 'http://172.16.1.146:8080/';
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
        floatingActionButton: FloatingActionButton.extended(
          onPressed: () {
            Navigator.push(
              context,
              MaterialPageRoute(builder: (context) => IP_Page()),
            );
          },
          label: Text("Enter IP"),
          icon: Icon(Icons.create),
          backgroundColor: Colors.red,
        ),
        bottomNavigationBar: BottomAppBar(
          color: Colors.grey[700],
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: <Widget>[
              ElevatedButton(
                onPressed: () {
                  print("stop pressed");
                },
                style: ElevatedButton.styleFrom(
                  primary: Colors.grey[500],
                  onPrimary: Colors.grey[400],
                  onSurface: Colors.grey[700],
                  elevation: 3.0,
                ),
                child: Text(
                  "STOP",
                  style: TextStyle(color: Colors.black),
                ),
              ),
              ElevatedButton(
                onPressed: () {
                  print("start pressed");
                  _sendLocations(dataPts);
                },
                style: ElevatedButton.styleFrom(
                  primary: Colors.grey[500],
                  onPrimary: Colors.grey[400],
                  onSurface: Colors.grey[700],
                  elevation: 3.0,
                ),
                //style: ElevatedButton.styleFrom(primary: Colors.grey[500]),
                child: Text(
                  "START",
                  style: TextStyle(color: Colors.black),
                ),
              ),
              ElevatedButton(
                onPressed: () {
                  print("reset pressed");
                  _reset();
                },
                style: ElevatedButton.styleFrom(
                  primary: Colors.grey[500],
                  onPrimary: Colors.grey[400],
                  onSurface: Colors.grey[700],
                  elevation: 3.0,
                ),
                //style: ElevatedButton.styleFrom(primary: Colors.grey[500]),
                child: Text(
                  "RESET",
                  style: TextStyle(color: Colors.black),
                ),
              )
            ],
          )),
    );
  }
}

class IP_Page extends StatefulWidget {
  IP_Page({Key key, this.title}) : super(key: key);
  final String title;

  @override
  _IP_PageState createState() => _IP_PageState();
}

class _IP_PageState extends State<IP_Page> {
  
 final _focusNode = FocusNode();
    
  @override
  void initState() {
    super.initState();
    _focusNode.addListener(() {

      print("Has focus: ${_focusNode.hasFocus}");
    });
  }

  @override
  void dispose() {
    _focusNode.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Container(
        color: Colors.grey[900],
          child: Center(
            child: TextField(
              focusNode: _focusNode,
              autocorrect: false,
              textInputAction: TextInputAction.send,
              decoration: InputDecoration(
                labelText: "IP Address",
                labelStyle: TextStyle(color: Colors.white),
                //hintText: "IP Address",
                // hintStyle: TextStyle(
                //   decorationColor: Colors.white,
                //   color: Colors.white,
                // ),
                border: OutlineInputBorder(
                  //borderSide: BorderSide(color: Colors.white),
                ),
                // suffixIcon: IconButton(
                //   icon: Icon(Icons.send),
                //   onPressed: 
                // ),
              ),
              // keyboardType: TextInputType.numberWithOptions(
              //   decimal: true,
              // ),
              style: TextStyle(
                color: Colors.white,
              ),
              onSubmitted: (text) {
                url = "http://$text/";
                print("ip address submitted: $text");
                Navigator.pop(context);
              }
            ),
          ),
        )
      );
  }
}
