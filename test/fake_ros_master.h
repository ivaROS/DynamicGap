#pragma once

#include <xmlrpcpp/XmlRpc.h>

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace dynamic_gap
{
    /**
    * \brief Minimal in-process ROS master for unit tests.
    *
    * roscpp registers services with the master as soon as the first ros::NodeHandle is
    * created (Planner owns one), and blocks until a master answers. This stub answers the
    * master and parameter-server calls roscpp makes, so tests can construct a Planner
    * without roscore. It keeps a flat parameter store; topics and services are accepted
    * but never connected.
    */
    class FakeRosMaster
    {
        public:
            using Handler = std::function<void(XmlRpc::XmlRpcValue & params, XmlRpc::XmlRpcValue & result)>;

            FakeRosMaster()
            {
                // registration calls: accept, nobody else on the graph
                for (const std::string & name : {"registerService", "unregisterService",
                                                 "registerPublisher", "unregisterPublisher",
                                                 "registerSubscriber", "unregisterSubscriber",
                                                 "subscribeParam", "unsubscribeParam"})
                {
                    addHandler(name, [name](XmlRpc::XmlRpcValue &, XmlRpc::XmlRpcValue & result)
                    {
                        XmlRpc::XmlRpcValue value;
                        if (name == "registerPublisher" || name == "registerSubscriber")
                            value.setSize(0);
                        else
                            value = 1;
                        reply(result, 1, value);
                    });
                }

                addHandler("getUri", [this](XmlRpc::XmlRpcValue &, XmlRpc::XmlRpcValue & result)
                {
                    XmlRpc::XmlRpcValue uri = this->uri();
                    reply(result, 1, uri);
                });

                addHandler("getParam", [this](XmlRpc::XmlRpcValue & params, XmlRpc::XmlRpcValue & result)
                {
                    std::lock_guard<std::mutex> lock(paramMutex_);
                    auto it = params_.find(std::string(params[1]));
                    XmlRpc::XmlRpcValue value;
                    if (it == params_.end())
                    {
                        value = 0;
                        reply(result, -1, value);
                    } else
                    {
                        value = it->second;
                        reply(result, 1, value);
                    }
                });

                addHandler("hasParam", [this](XmlRpc::XmlRpcValue & params, XmlRpc::XmlRpcValue & result)
                {
                    std::lock_guard<std::mutex> lock(paramMutex_);
                    XmlRpc::XmlRpcValue value = (params_.count(std::string(params[1])) > 0);
                    reply(result, 1, value);
                });

                addHandler("setParam", [this](XmlRpc::XmlRpcValue & params, XmlRpc::XmlRpcValue & result)
                {
                    std::lock_guard<std::mutex> lock(paramMutex_);
                    params_[std::string(params[1])] = params[2];
                    XmlRpc::XmlRpcValue value = 0;
                    reply(result, 1, value);
                });

                addHandler("deleteParam", [this](XmlRpc::XmlRpcValue & params, XmlRpc::XmlRpcValue & result)
                {
                    std::lock_guard<std::mutex> lock(paramMutex_);
                    params_.erase(std::string(params[1]));
                    XmlRpc::XmlRpcValue value = 0;
                    reply(result, 1, value);
                });

                addHandler("searchParam", [](XmlRpc::XmlRpcValue &, XmlRpc::XmlRpcValue & result)
                {
                    XmlRpc::XmlRpcValue value = 0;
                    reply(result, -1, value);
                });
            }

            ~FakeRosMaster() { stop(); }

            /**
            * \brief Bind to a free local port and serve requests on a background thread
            * \return true on success
            */
            bool start()
            {
                if (!server_.bindAndListen(0))
                    return false;
                running_ = true;
                thread_ = std::thread([this]()
                {
                    while (running_)
                        server_.work(0.05);
                });
                return true;
            }

            void stop()
            {
                if (!running_)
                    return;
                running_ = false;
                if (thread_.joinable())
                    thread_.join();
                server_.shutdown();
            }

            std::string uri() const { return "http://localhost:" + std::to_string(const_cast<XmlRpc::XmlRpcServer &>(server_).get_port()) + "/"; }

        private:
            class Method : public XmlRpc::XmlRpcServerMethod
            {
                public:
                    Method(const std::string & name, XmlRpc::XmlRpcServer * server, Handler handler)
                        : XmlRpc::XmlRpcServerMethod(name, server), handler_(std::move(handler)) {}

                    void execute(XmlRpc::XmlRpcValue & params, XmlRpc::XmlRpcValue & result) override { handler_(params, result); }

                private:
                    Handler handler_;
            };

            static void reply(XmlRpc::XmlRpcValue & result, int code, XmlRpc::XmlRpcValue & value)
            {
                result.setSize(3);
                result[0] = code;
                result[1] = std::string("");
                result[2] = value;
            }

            void addHandler(const std::string & name, Handler handler)
            {
                methods_.emplace_back(new Method(name, &server_, std::move(handler)));
            }

            XmlRpc::XmlRpcServer server_;
            std::vector<std::unique_ptr<Method>> methods_;
            std::thread thread_;
            std::atomic<bool> running_{false};

            std::mutex paramMutex_;
            std::map<std::string, XmlRpc::XmlRpcValue> params_;
    };
}
