struct position {
  double x;
  double y;
};
class odom {
private:
  double x = 0;
  double y = 0;
public:
  position odomoutputs();
  void reset();
  void setStarting(double ax, double ay);
};
odom odometry;