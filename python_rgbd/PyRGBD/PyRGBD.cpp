#include "PyRGBD.h"
#include "ndarray_converter.h"
#include <rgbd/image.h>
#include <rgbd/types.h>
#include <rgbd/client.h>

#include <ros/init.h>

namespace py = pybind11;


PYBIND11_MODULE(PyRGBD, m)
{
    NDArrayConverter::init_numpy();

    m.doc() = "RGBD Python wrapper"; // optional module docstring
    m.attr("__version__") = "0.1.0";

    // --------------------
    // Image
    // --------------------
    py::class_<rgbd::Image> image(m, "Image");
    image.def(py::init<>());
    image.def("getDepthImage", &rgbd::Image::getDepthImage);
    image.def("getRGBImage", &rgbd::Image::getRGBImage);
    image.def("getFrameId", &rgbd::Image::getFrameId);
    image.def("getTimestamp", &rgbd::Image::getTimestamp);
    image.def("clone", &rgbd::Image::clone);

    image.def("__copy__", [](const rgbd::Image& self)
    {
        return rgbd::Image(self);
    });
    image.def("__deepcopy__", [](const rgbd::Image& self, py::dict)
    {
        return rgbd::Image(self);
    }, py::arg("memo"));


    // --------------------
    // Client
    // --------------------
    py::class_<rgbd::Client> client(m, "Client");
    client.def(py::init<>());
    client.def("initialize", &rgbd::Client::initialize, py::arg("server_name"), py::arg("timeout")=5.0);
    client.def("initialized", &rgbd::Client::initialized);
    client.def("nextImage",  (bool (rgbd::Client::*)(rgbd::Image&)) &rgbd::Client::nextImage, py::arg("image"));

    // --------------------
    // Fucntions
    // --------------------
    m.def("roscpp_init", [](std::vector<std::string> args, std::string node_name) {
        std::vector<char*> cargs{};
        for(auto& arg : args)
            cargs.push_back(&arg.front());
        int size = cargs.size();
        ros::init(size, cargs.data(), node_name);
    });
}
