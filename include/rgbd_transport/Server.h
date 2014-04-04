#ifndef _Server_H_
#define _Server_H_

#include <ros/ros.h>

class RGBDImage;

class Server {

public:

    Server();

    virtual ~Server();

    void initialize(const std::string& name);

    void send(const RGBDImage& image);

};

#endif
