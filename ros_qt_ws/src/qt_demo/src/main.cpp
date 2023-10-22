#include "mainwindow.h"

#include <QApplication>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication a(argc, argv);
    MainWindow w;
    auto node = std::make_shared<rclcpp::Node>("hello_world_node");
    RCLCPP_INFO(node->get_logger(), "Hello world!");
    rclcpp::spin(node);
    w.show();
    rclcpp::shutdown();
    return a.exec();
}
