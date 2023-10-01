#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_projection/LocalCartesian.h>

void getScaleChangeSquareRight(double scale, lanelet::BasicPoint3d point1, lanelet::BasicPoint3d point2, lanelet::BasicPoint3d point3, lanelet::BasicPoint3d point4);
void getScaleChangeSquareLeft(double scale, lanelet::BasicPoint3d point1, lanelet::BasicPoint3d point2, lanelet::BasicPoint3d point3, lanelet::BasicPoint3d point4);
void getCartesianCoordinateKonkuk(lanelet::BasicPoint3d point);
void getCartesianCoordinateKcity(lanelet::BasicPoint3d point);

lanelet::projection::UtmProjector utm_projector_konkuk(lanelet::Origin({37.5421000,127.0770500}));
lanelet::projection::UtmProjector utm_projector_k_city(lanelet::Origin({37.23855064,126.77253029}));
lanelet::projection::LocalCartesianProjector cart_projector_konkuk(lanelet::Origin({37.5421000,127.0770500}));
lanelet::projection::LocalCartesianProjector cart_projector_k_city(lanelet::Origin({37.23855064,126.77253029}));

int main(int argc, char **argv){
    std::string node_name = "converter";
    ros::init(argc, argv, node_name);

    lanelet::GPSPoint a1,a2,a3,a4,b1,b2,b3,b4,c1,c2,c3,c4;

    // konkuk

    a1.lat = 37.54019466442; a1.lon = 127.07984288397;
    a2.lat = 37.54020636178; a2.lon = 127.07982746126;
    a3.lat = 37.54023241499; a3.lon = 127.07986367108;
    a4.lat = 37.54021965424; a4.lon = 127.07987842323;

    b1.lat = 37.54029337563; b1.lon = 127.08002299421;
    b2.lat = 37.54030507298; b2.lon = 127.08000757151;
    b3.lat = 37.54033112615; b3.lon = 127.08004378133;
    b4.lat = 37.54031836542; b4.lon = 127.08005853348;

    c1.lat = 37.54031783925; c1.lon = 127.08006345451;
    c2.lat = 37.54033022867; c2.lon = 127.08004919556;
    c3.lat = 37.54035455165; c3.lon = 127.08008555085;
    c4.lat = 37.54034156023; c4.lon = 127.08009884832;

    lanelet::BasicPoint3d utm_a1,utm_a2,utm_a3,utm_a4,utm_b1,utm_b2,utm_b3,utm_b4,utm_c1,utm_c2,utm_c3,utm_c4;

    utm_a1 = utm_projector_k_city.forward(a1);
    utm_a2 = utm_projector_k_city.forward(a2);
    utm_a3 = utm_projector_k_city.forward(a3);
    utm_a4 = utm_projector_k_city.forward(a4);

    utm_b1 = utm_projector_k_city.forward(b1);
    utm_b2 = utm_projector_k_city.forward(b2);
    utm_b3 = utm_projector_k_city.forward(b3);
    utm_b4 = utm_projector_k_city.forward(b4);
    
    utm_c1 = utm_projector_k_city.forward(c1);
    utm_c2 = utm_projector_k_city.forward(c2);
    utm_c3 = utm_projector_k_city.forward(c3);
    utm_c4 = utm_projector_k_city.forward(c4);

    lanelet::BasicPoint3d cart_a1,cart_a2,cart_a3,cart_a4,cart_b1,cart_b2,cart_b3,cart_b4,cart_c1,cart_c2,cart_c3,cart_c4;

    cart_a1 = cart_projector_konkuk.forward(a1);
    cart_a2 = cart_projector_konkuk.forward(a2);
    cart_a3 = cart_projector_konkuk.forward(a3);
    cart_a4 = cart_projector_konkuk.forward(a4);

    cart_b1 = cart_projector_konkuk.forward(b1);
    cart_b2 = cart_projector_konkuk.forward(b2);
    cart_b3 = cart_projector_konkuk.forward(b3);
    cart_b4 = cart_projector_konkuk.forward(b4);
    
    cart_c1 = cart_projector_konkuk.forward(c1);
    cart_c2 = cart_projector_konkuk.forward(c2);
    cart_c3 = cart_projector_konkuk.forward(c3);
    cart_c4 = cart_projector_konkuk.forward(c4);

    // left extension
    std::cout << "Left Cartesian Result" << std::endl;
    std::cout << "\n point1 \n" << std::endl;
    getScaleChangeSquareLeft(0.8, cart_a1, cart_a2, cart_a3, cart_a4);
    std::cout << "\n point2 \n" << std::endl;
    getScaleChangeSquareLeft(0.8, cart_b1, cart_b2, cart_b3, cart_b4);
    std::cout << "\n point3 \n" << std::endl;
    getScaleChangeSquareLeft(0.8, cart_c1, cart_c2, cart_c3, cart_c4);

    // right extension
    std::cout << "\n Right Cartesian Result" << std::endl;
    std::cout << "\n point1 \n" << std::endl;
    getScaleChangeSquareRight(0.8, cart_a1, cart_a2, cart_a3, cart_a4);
    std::cout << "\n point2 \n" << std::endl;
    getScaleChangeSquareRight(0.8, cart_b1, cart_b2, cart_b3, cart_b4);
    std::cout << "\n point3 \n" << std::endl;
    getScaleChangeSquareRight(0.8, cart_c1, cart_c2, cart_c3, cart_c4);

    lanelet::BasicPoint3d kcity_slot0, kcity_slot1, kcity_slot2, kcity_slot0_SOTA, kcity_delivery_slot0, kcity_delivery_slot1, kcity_delivery_slot2, kcity_delivery_A, konkuk_mungwa_slot_0, konkuk_mungwa_slot_1, konkuk_mungwa_slot_2;

    kcity_slot0.x() = 62.7; kcity_slot0.y() = 101.5; 
    kcity_slot1.x() = 60.0; kcity_slot1.y() = 96.8;
    kcity_slot2.x() = 57.9; kcity_slot2.y() = 93.1;

    return 0;
}

void getScaleChangeSquareRight(double scale, lanelet::BasicPoint3d point1, lanelet::BasicPoint3d point2, lanelet::BasicPoint3d point3, lanelet::BasicPoint3d point4){

    double center_x = (point1.x() + point2.x() + point3.x() + point4.x()) / 4.;
    double center_y = (point1.y() + point2.y() + point3.y() + point4.y()) / 4.;

    // std::cout << "left bottom, right bottom, right top, left top" << std::endl;
    
    double left_bottom_x = center_x + scale * (point1.x()-center_x);
    double left_bottom_y = center_y + scale * (point1.y()-center_y);
    double right_bottom_x = center_x + scale * (point2.x()-center_x);
    double right_bottom_y = center_y + scale * (point2.y()-center_y);
    double right_top_x = center_x + scale * (point3.x()-center_x);
    double right_top_y = center_y + scale * (point3.y()-center_y);
    double left_top_x = center_x + scale * (point4.x()-center_x);
    double left_top_y = center_y + scale * (point4.y()-center_y);

    double left_bottom_translate_x = (right_bottom_x + 3 * left_bottom_x) / 4.;
    double left_bottom_translate_y = (right_bottom_y + 3 * left_bottom_y) / 4.;
    double left_top_translate_x = (right_top_x + 3 * left_top_x) / 4.;
    double left_top_translate_y = (right_top_y + 3 * left_top_y) / 4.;

    double right_bottom_translate_x = (5 * right_bottom_x - 1.5 * left_bottom_x) / 3.5;
    double right_bottom_translate_y = (5 * right_bottom_y - 1.5 * left_bottom_y) / 3.5;
    double right_top_translate_x = (5 * right_top_x - 1.5 * left_top_x) / 3.5;
    double right_top_translate_y = (5 * right_top_y - 1.5 * left_top_y) / 3.5;

    std::cout <<"(" << left_bottom_translate_x << ", " << left_bottom_translate_y <<"), " <<
                "(" << right_bottom_translate_x << ", " << right_bottom_translate_y <<"), " <<
                "(" << right_top_translate_x << ", " << right_top_translate_y <<"), " <<
                "(" << left_top_translate_x << ", " << left_top_translate_y <<") \n" <<
                "center point: (" << (left_bottom_translate_x + right_bottom_translate_x + right_top_translate_x + left_top_translate_x)/4. << ", "
                << (left_bottom_translate_y + right_bottom_translate_y + right_top_translate_y + left_top_translate_y)/4. << ")" <<
                std::endl;

    std::cout << left_bottom_translate_x <<" "<< left_bottom_translate_y <<" "<< right_bottom_translate_x <<" "<< right_bottom_translate_y <<" "<< right_top_translate_x <<" "<< right_top_translate_y <<" "<< left_top_translate_x <<" "<< left_top_translate_y << std::endl;

    // std::cout << sqrt(pow(scale * (point1.x()-center_x) - scale * (point2.x()-center_x),2) + pow(scale * (point1.y()-center_y) - scale * (point2.y()-center_y),2));
}

void getScaleChangeSquareLeft(double scale, lanelet::BasicPoint3d point1, lanelet::BasicPoint3d point2, lanelet::BasicPoint3d point3, lanelet::BasicPoint3d point4){

    double center_x = (point1.x() + point2.x() + point3.x() + point4.x()) / 4.;
    double center_y = (point1.y() + point2.y() + point3.y() + point4.y()) / 4.;

    // std::cout << "left bottom, right bottom, right top, left top" << std::endl;
    
    double left_bottom_x = center_x + scale * (point1.x()-center_x);
    double left_bottom_y = center_y + scale * (point1.y()-center_y);
    double right_bottom_x = center_x + scale * (point2.x()-center_x);
    double right_bottom_y = center_y + scale * (point2.y()-center_y);
    double right_top_x = center_x + scale * (point3.x()-center_x);
    double right_top_y = center_y + scale * (point3.y()-center_y);
    double left_top_x = center_x + scale * (point4.x()-center_x);
    double left_top_y = center_y + scale * (point4.y()-center_y);

    double left_bottom_translate_x = (5 * right_bottom_x - 1.5 * left_bottom_x) / 3.5;
    double left_bottom_translate_y = (5 * right_bottom_y - 1.5 * left_bottom_y) / 3.5;
    double left_top_translate_x = (5 * right_top_x - 1.5 * left_top_x) / 3.5;
    double left_top_translate_y = (5 * right_top_y - 1.5 * left_top_y) / 3.5;

    double right_bottom_translate_x = (right_bottom_x + 3 * left_bottom_x) / 4.;
    double right_bottom_translate_y = (right_bottom_y + 3 * left_bottom_y) / 4.;
    double right_top_translate_x = (right_top_x + 3 * left_top_x) / 4.;
    double right_top_translate_y = (right_top_y + 3 * left_top_y) / 4.;

    std::cout <<"(" << left_bottom_translate_x << ", " << left_bottom_translate_y <<"), " <<
                "(" << right_bottom_translate_x << ", " << right_bottom_translate_y <<"), " <<
                "(" << right_top_translate_x << ", " << right_top_translate_y <<"), " <<
                "(" << left_top_translate_x << ", " << left_top_translate_y <<") \n" <<
                "center point: (" << (left_bottom_translate_x + right_bottom_translate_x + right_top_translate_x + left_top_translate_x)/4. << ", "
                << (left_bottom_translate_y + right_bottom_translate_y + right_top_translate_y + left_top_translate_y)/4. << ")" <<
                std::endl;

    std::cout << left_bottom_translate_x <<" "<< left_bottom_translate_y <<" "<< right_bottom_translate_x <<" "<< right_bottom_translate_y <<" "<< right_top_translate_x <<" "<< right_top_translate_y <<" "<< left_top_translate_x <<" "<< left_top_translate_y << std::endl;

    // std::cout << sqrt(pow(scale * (point1.x()-center_x) - scale * (point2.x()-center_x),2) + pow(scale * (point1.y()-center_y) - scale * (point2.y()-center_y),2));
}

void getCartesianCoordinateKonkuk(lanelet::BasicPoint3d point){
    lanelet::GPSPoint GPSPoint = utm_projector_konkuk.reverse(point);
    lanelet::BasicPoint3d cart_point = cart_projector_konkuk.forward(GPSPoint);
    std::cout<< "\n(" << cart_point.x() << ", "<< cart_point.y() <<")" << std::endl;
}

void getCartesianCoordinateKcity(lanelet::BasicPoint3d point){
    lanelet::GPSPoint GPSPoint = utm_projector_k_city.reverse(point);
    lanelet::BasicPoint3d cart_point = cart_projector_k_city.forward(GPSPoint);
    std::cout<< "\n(" << cart_point.x() << ", "<< cart_point.y() <<")" << std::endl;
}
