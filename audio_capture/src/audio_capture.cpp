#include <stdio.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <boost/thread.hpp>

#include <ros/ros.h>

#include "audio_common_msgs/AudioData.h"

namespace audio_transport
{
  class RosGstCapture
  {
    public:
      RosGstCapture()
      {
        _bitrate = 192;

        std::string dst_type;

        // Need to encoding or publish raw wave data
        ros::param::param<std::string>("~format", _format, "mp3");

        // The bitrate at which to encode the audio
        ros::param::param<int>("~bitrate", _bitrate, 192);

        // only available for raw data
        ros::param::param<int>("~channels", _channels, 1);
        ros::param::param<int>("~depth", _depth, 16);
        ros::param::param<int>("~sample_rate", _sample_rate, 16000);

        // The destination of the audio
        ros::param::param<std::string>("~dst", dst_type, "appsink");

        // The source of the audio
        //ros::param::param<std::string>("~src", source_type, "alsasrc");
        std::string device;
        ros::param::param<std::string>("~device", device, "");

        std::string topic;
        ros::param::param<std::string>("~topic", topic, "audio");

        _pub = _nh.advertise<audio_common_msgs::AudioData>(topic, 10, true);

        _loop = g_main_loop_new(NULL, false);
        _pipeline = gst_pipeline_new("ros_pipeline");
        _bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline));
        gst_bus_add_signal_watch(_bus);
        g_signal_connect(_bus, "message::error",
                         G_CALLBACK(onMessage), this);
        g_object_unref(_bus);

        // We create the sink first, just for convenience
        if (dst_type == "appsink")
        {
          _sink = gst_element_factory_make("appsink", "sink");
          g_object_set(G_OBJECT(_sink), "emit-signals", true, NULL);
          g_object_set(G_OBJECT(_sink), "max-buffers", 100, NULL);
          g_signal_connect( G_OBJECT(_sink), "new-sample",
                            G_CALLBACK(onNewBuffer), this);
        }
        else
        {
          printf("file sink\n");
          _sink = gst_element_factory_make("filesink", "sink");
          g_object_set( G_OBJECT(_sink), "location", dst_type.c_str(), NULL);
        }

        _source = gst_element_factory_make("alsasrc", "source");
        // if device isn't specified, it will use the default which is
        // the alsa default source.
        // A valid device will be of the foram hw:0,0 with other numbers
        // than 0 and 0 as are available.
        if (device != "")
        {
          // ghcar *gst_device = device.c_str();
          g_object_set(G_OBJECT(_source), "device", device.c_str(), NULL);
        }

        _filter = gst_element_factory_make("capsfilter", "filter");
        {
          GstCaps *caps;
          caps = gst_caps_new_simple("audio/x-raw",
                        //      "channels", G_TYPE_INT, _channels,
                        //      "depth",    G_TYPE_INT, _depth,
                              "rate",     G_TYPE_INT, _sample_rate,
                        //       "signed",   G_TYPE_BOOLEAN, TRUE,
                              NULL);
          g_object_set( G_OBJECT(_filter), "caps", caps, NULL);
          gst_caps_unref(caps);
        }

        _convert = gst_element_factory_make("audioconvert", "convert");
        if (!_convert) {
      	  ROS_ERROR_STREAM("Failed to create audioconvert element");
      	  exitOnMainThread(1);
        }

        gboolean link_ok;

        if (_format == "mp3"){
          _encode = gst_element_factory_make("lamemp3enc", "encoder");
          if (!_encode) {
        	  ROS_ERROR_STREAM("Failed to create encoder element");
        	  exitOnMainThread(1);
          }
          g_object_set( G_OBJECT(_encode), "quality", 2.0, NULL);
          g_object_set( G_OBJECT(_encode), "bitrate", _bitrate, NULL);

          gst_bin_add_many( GST_BIN(_pipeline), _source, _filter, _convert, _encode, _sink, NULL);
          link_ok = gst_element_link_many(_source, _filter, _convert, _encode, _sink, NULL);
        } else if (_format == "wave") {
          GstCaps *caps;
          caps = gst_caps_new_simple("audio/x-raw",
                                     "channels", G_TYPE_INT, _channels,
                                     "width",    G_TYPE_INT, _depth,
                                     "depth",    G_TYPE_INT, _depth,
                                     "rate",     G_TYPE_INT, _sample_rate,
                                     "signed",   G_TYPE_BOOLEAN, TRUE,
                                     NULL);

          g_object_set( G_OBJECT(_sink), "caps", caps, NULL);
          gst_caps_unref(caps);
          gst_bin_add_many( GST_BIN(_pipeline), _source, _sink, NULL);
          link_ok = gst_element_link_many( _source, _sink, NULL);
        } else {
          ROS_ERROR_STREAM("format must be \"wave\" or \"mp3\"");
          exitOnMainThread(1);
        }
        /*}
        else
        {
          _sleep_time = 10000;
          _source = gst_element_factory_make("filesrc", "source");
          g_object_set(G_OBJECT(_source), "location", source_type.c_str(), NULL);

          gst_bin_add_many( GST_BIN(_pipeline), _source, _sink, NULL);
          gst_element_link_many(_source, _sink, NULL);
        }
        */

        if (!link_ok) {
          ROS_ERROR_STREAM("Unsupported media type.");
          exitOnMainThread(1);
        }

        gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);

        _gst_thread = boost::thread( boost::bind(g_main_loop_run, _loop) );
      }

      ~RosGstCapture()
      {
        g_main_loop_quit(_loop);
        gst_element_set_state(_pipeline, GST_STATE_NULL);
        gst_object_unref(_pipeline);
        g_main_loop_unref(_loop);
      }

      void exitOnMainThread(int code)
      {
        exit(code);
      }

      void publish( const audio_common_msgs::AudioData &msg )
      {
        _pub.publish(msg);
      }

      static GstFlowReturn onNewBuffer (GstAppSink *appsink, gpointer userData)
      {
        RosGstCapture *server = reinterpret_cast<RosGstCapture*>(userData);
        GstMapInfo map;

        GstSample *sample;
        g_signal_emit_by_name(appsink, "pull-sample", &sample);

        GstBuffer *buffer = gst_sample_get_buffer(sample);

        audio_common_msgs::AudioData msg;
        gst_buffer_map(buffer, &map, GST_MAP_READ);
        msg.data.resize( map.size );

        memcpy( &msg.data[0], map.data, map.size );

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        server->publish(msg);

        return GST_FLOW_OK;
      }

      static gboolean onMessage (GstBus *bus, GstMessage *message, gpointer userData)
      {
        RosGstCapture *server = reinterpret_cast<RosGstCapture*>(userData);
        GError *err;
        gchar *debug;

        gst_message_parse_error(message, &err, &debug);
        ROS_ERROR_STREAM("gstreamer: " << err->message);
        g_error_free(err);
        g_free(debug);
        g_main_loop_quit(server->_loop);
        server->exitOnMainThread(1);
        return FALSE;
      }

    private:
      ros::NodeHandle _nh;
      ros::Publisher _pub;

      boost::thread _gst_thread;

      GstElement *_pipeline, *_source, *_filter, *_sink, *_convert, *_encode;
      GstBus *_bus;
      int _bitrate, _channels, _depth, _sample_rate;
      GMainLoop *_loop;
      std::string _format;
  };
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "audio_capture", ros::init_options::AnonymousName);
  gst_init(&argc, &argv);

  audio_transport::RosGstCapture server;
  ros::spin();
}
