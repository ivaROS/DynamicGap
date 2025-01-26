//#line 2 "/opt/ros/noetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"
// *********************************************************
//
// File autogenerated for the agent_path_prediction package
// by the dynamic_reconfigure package.
// Please do not edit.
//
// ********************************************************/

#ifndef __agent_path_prediction__AGENTPATHPREDICTIONCONFIG_H__
#define __agent_path_prediction__AGENTPATHPREDICTIONCONFIG_H__

#if __cplusplus >= 201103L
#define DYNAMIC_RECONFIGURE_FINAL final
#else
#define DYNAMIC_RECONFIGURE_FINAL
#endif

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace agent_path_prediction
{
  class AgentPathPredictionConfigStatics;

  class AgentPathPredictionConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l,
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }
      virtual ~AbstractParamDescription() = default;

      virtual void clamp(AgentPathPredictionConfig &config, const AgentPathPredictionConfig &max, const AgentPathPredictionConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const AgentPathPredictionConfig &config1, const AgentPathPredictionConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, AgentPathPredictionConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const AgentPathPredictionConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, AgentPathPredictionConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const AgentPathPredictionConfig &config) const = 0;
      virtual void getValue(const AgentPathPredictionConfig &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template <class T>
    class ParamDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string a_name, std::string a_type, uint32_t a_level,
          std::string a_description, std::string a_edit_method, T AgentPathPredictionConfig::* a_f) :
        AbstractParamDescription(a_name, a_type, a_level, a_description, a_edit_method),
        field(a_f)
      {}

      T AgentPathPredictionConfig::* field;

      virtual void clamp(AgentPathPredictionConfig &config, const AgentPathPredictionConfig &max, const AgentPathPredictionConfig &min) const override
      {
        if (config.*field > max.*field)
          config.*field = max.*field;

        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const AgentPathPredictionConfig &config1, const AgentPathPredictionConfig &config2) const override
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, AgentPathPredictionConfig &config) const override
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const AgentPathPredictionConfig &config) const override
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, AgentPathPredictionConfig &config) const override
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const AgentPathPredictionConfig &config) const override
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const AgentPathPredictionConfig &config, boost::any &val) const override
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription : public dynamic_reconfigure::Group
    {
      public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
      {
        name = n;
        type = t;
        parent = p;
        state = s;
        id = i;
      }

      virtual ~AbstractGroupDescription() = default;

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
      virtual void updateParams(boost::any &cfg, AgentPathPredictionConfig &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); ++i)
        {
          parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
        }
      }
    };

    typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template<class T, class PT>
    class GroupDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string a_name, std::string a_type, int a_parent, int a_id, bool a_s, T PT::* a_f) : AbstractGroupDescription(a_name, a_type, a_parent, a_id, a_s), field(a_f)
      {
      }

      GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
      {
        parameters = g.parameters;
        abstract_parameters = g.abstract_parameters;
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const override
      {
        PT* config = boost::any_cast<PT*>(cfg);
        if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
          return false;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          if(!(*i)->fromMessage(msg, n))
            return false;
        }

        return true;
      }

      virtual void setInitialState(boost::any &cfg) const override
      {
        PT* config = boost::any_cast<PT*>(cfg);
        T* group = &((*config).*field);
        group->state = state;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }

      virtual void updateParams(boost::any &cfg, AgentPathPredictionConfig &top) const override
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const override
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T PT::* field;
      std::vector<AgentPathPredictionConfig::AbstractGroupDescriptionConstPtr> groups;
    };

class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(AgentPathPredictionConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("velscale_lower"==(*_i)->name){velscale_lower = boost::any_cast<double>(val);}
        if("velscale_nominal"==(*_i)->name){velscale_nominal = boost::any_cast<double>(val);}
        if("velscale_higher"==(*_i)->name){velscale_higher = boost::any_cast<double>(val);}
        if("velscale_angle"==(*_i)->name){velscale_angle = boost::any_cast<double>(val);}
        if("velscale_mul"==(*_i)->name){velscale_mul = boost::any_cast<double>(val);}
        if("velobs_mul"==(*_i)->name){velobs_mul = boost::any_cast<double>(val);}
        if("velobs_min_rad"==(*_i)->name){velobs_min_rad = boost::any_cast<double>(val);}
        if("velobs_max_rad"==(*_i)->name){velobs_max_rad = boost::any_cast<double>(val);}
        if("velobs_max_rad_time"==(*_i)->name){velobs_max_rad_time = boost::any_cast<double>(val);}
        if("velobs_use_ang"==(*_i)->name){velobs_use_ang = boost::any_cast<bool>(val);}
        if("robot_frame_id"==(*_i)->name){robot_frame_id = boost::any_cast<std::string>(val);}
      }
    }

    double velscale_lower;
double velscale_nominal;
double velscale_higher;
double velscale_angle;
double velscale_mul;
double velobs_mul;
double velobs_min_rad;
double velobs_max_rad;
double velobs_max_rad_time;
bool velobs_use_ang;
std::string robot_frame_id;

    bool state;
    std::string name;

    
}groups;



//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double velscale_lower;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double velscale_nominal;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double velscale_higher;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double velscale_angle;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double velscale_mul;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double velobs_mul;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double velobs_min_rad;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double velobs_max_rad;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double velobs_max_rad_time;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      bool velobs_use_ang;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      std::string robot_frame_id;
//#line 231 "/opt/ros/noetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        if ((*i)->fromMessage(msg, *this))
          count++;

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
      {
        if ((*i)->id == 0)
        {
          boost::any n = boost::any(this);
          (*i)->updateParams(n, *this);
          (*i)->fromMessage(msg, n);
        }
      }

      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("AgentPathPredictionConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        if((*i)->id == 0)
        {
          (*i)->toMessage(msg, *this);
        }
      }
    }

    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      __toMessage__(msg, __param_descriptions__, __group_descriptions__);
    }

    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->fromServer(nh, *this);

      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
        if (!setup && (*i)->id == 0) {
          setup = true;
          boost::any n = boost::any(this);
          (*i)->setInitialState(n);
        }
      }
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const AgentPathPredictionConfig &__max__ = __getMax__();
      const AgentPathPredictionConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const AgentPathPredictionConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->calcLevel(level, config, *this);
      return level;
    }

    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const AgentPathPredictionConfig &__getDefault__();
    static const AgentPathPredictionConfig &__getMax__();
    static const AgentPathPredictionConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();

  private:
    static const AgentPathPredictionConfigStatics *__get_statics__();
  };

  template <> // Max and min are ignored for strings.
  inline void AgentPathPredictionConfig::ParamDescription<std::string>::clamp(AgentPathPredictionConfig &config, const AgentPathPredictionConfig &max, const AgentPathPredictionConfig &min) const
  {
    (void) config;
    (void) min;
    (void) max;
    return;
  }

  class AgentPathPredictionConfigStatics
  {
    friend class AgentPathPredictionConfig;

    AgentPathPredictionConfigStatics()
    {
AgentPathPredictionConfig::GroupDescription<AgentPathPredictionConfig::DEFAULT, AgentPathPredictionConfig> Default("Default", "", 0, 0, true, &AgentPathPredictionConfig::groups);
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.velscale_lower = 0.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.velscale_lower = 1.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.velscale_lower = 0.8;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<double>("velscale_lower", "double", 0, "agent slow-down velocity multiplier for velcity-scale calculation", "", &AgentPathPredictionConfig::velscale_lower)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<double>("velscale_lower", "double", 0, "agent slow-down velocity multiplier for velcity-scale calculation", "", &AgentPathPredictionConfig::velscale_lower)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.velscale_nominal = 0.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.velscale_nominal = 10.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.velscale_nominal = 1.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<double>("velscale_nominal", "double", 0, "agent nominal velocity multiplier for velcity-scale calculation", "", &AgentPathPredictionConfig::velscale_nominal)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<double>("velscale_nominal", "double", 0, "agent nominal velocity multiplier for velcity-scale calculation", "", &AgentPathPredictionConfig::velscale_nominal)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.velscale_higher = 1.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.velscale_higher = 10.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.velscale_higher = 1.2;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<double>("velscale_higher", "double", 0, "agent speed-up velocity multiplier for velcity-scale calculation", "", &AgentPathPredictionConfig::velscale_higher)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<double>("velscale_higher", "double", 0, "agent speed-up velocity multiplier for velcity-scale calculation", "", &AgentPathPredictionConfig::velscale_higher)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.velscale_angle = 0.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.velscale_angle = 3.14;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.velscale_angle = 0.1;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<double>("velscale_angle", "double", 0, "deviation angle for agent position predictions for velcity-scale calculation", "", &AgentPathPredictionConfig::velscale_angle)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<double>("velscale_angle", "double", 0, "deviation angle for agent position predictions for velcity-scale calculation", "", &AgentPathPredictionConfig::velscale_angle)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.velscale_mul = 0.001;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.velscale_mul = 10.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.velscale_mul = 1.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<double>("velscale_mul", "double", 0, "multiplier for agent velocities for velocity-scale calculation", "", &AgentPathPredictionConfig::velscale_mul)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<double>("velscale_mul", "double", 0, "multiplier for agent velocities for velocity-scale calculation", "", &AgentPathPredictionConfig::velscale_mul)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.velobs_mul = 0.001;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.velobs_mul = 10.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.velobs_mul = 1.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<double>("velobs_mul", "double", 0, "multiplier for agent velocities for velocity-obstacle calculation", "", &AgentPathPredictionConfig::velobs_mul)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<double>("velobs_mul", "double", 0, "multiplier for agent velocities for velocity-obstacle calculation", "", &AgentPathPredictionConfig::velobs_mul)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.velobs_min_rad = 0.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.velobs_min_rad = 10.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.velobs_min_rad = 0.25;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<double>("velobs_min_rad", "double", 0, "minimum radius for velcoity-obstacle calculation", "", &AgentPathPredictionConfig::velobs_min_rad)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<double>("velobs_min_rad", "double", 0, "minimum radius for velcoity-obstacle calculation", "", &AgentPathPredictionConfig::velobs_min_rad)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.velobs_max_rad = 0.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.velobs_max_rad = 10.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.velobs_max_rad = 0.75;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<double>("velobs_max_rad", "double", 0, "maxium radius for volocity-obstacle calculation", "", &AgentPathPredictionConfig::velobs_max_rad)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<double>("velobs_max_rad", "double", 0, "maxium radius for volocity-obstacle calculation", "", &AgentPathPredictionConfig::velobs_max_rad)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.velobs_max_rad_time = 0.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.velobs_max_rad_time = 60.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.velobs_max_rad_time = 4.0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<double>("velobs_max_rad_time", "double", 0, "time for maximum radius for velocity-obstacle calculation", "", &AgentPathPredictionConfig::velobs_max_rad_time)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<double>("velobs_max_rad_time", "double", 0, "time for maximum radius for velocity-obstacle calculation", "", &AgentPathPredictionConfig::velobs_max_rad_time)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.velobs_use_ang = 0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.velobs_use_ang = 1;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.velobs_use_ang = 1;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<bool>("velobs_use_ang", "bool", 0, "wheter to use angular velocity for velocity-obstacle calculation", "", &AgentPathPredictionConfig::velobs_use_ang)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<bool>("velobs_use_ang", "bool", 0, "wheter to use angular velocity for velocity-obstacle calculation", "", &AgentPathPredictionConfig::velobs_use_ang)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.robot_frame_id = "";
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.robot_frame_id = "";
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.robot_frame_id = "base_footprint";
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<std::string>("robot_frame_id", "str", 0, "base frame id for robot", "", &AgentPathPredictionConfig::robot_frame_id)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(AgentPathPredictionConfig::AbstractParamDescriptionConstPtr(new AgentPathPredictionConfig::ParamDescription<std::string>("robot_frame_id", "str", 0, "base frame id for robot", "", &AgentPathPredictionConfig::robot_frame_id)));
//#line 246 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.convertParams();
//#line 246 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __group_descriptions__.push_back(AgentPathPredictionConfig::AbstractGroupDescriptionConstPtr(new AgentPathPredictionConfig::GroupDescription<AgentPathPredictionConfig::DEFAULT, AgentPathPredictionConfig>(Default)));
//#line 369 "/opt/ros/noetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

      for (std::vector<AgentPathPredictionConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__);
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__);
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__);
    }
    std::vector<AgentPathPredictionConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<AgentPathPredictionConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    AgentPathPredictionConfig __max__;
    AgentPathPredictionConfig __min__;
    AgentPathPredictionConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const AgentPathPredictionConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static AgentPathPredictionConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &AgentPathPredictionConfig::__getDescriptionMessage__()
  {
    return __get_statics__()->__description_message__;
  }

  inline const AgentPathPredictionConfig &AgentPathPredictionConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }

  inline const AgentPathPredictionConfig &AgentPathPredictionConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }

  inline const AgentPathPredictionConfig &AgentPathPredictionConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }

  inline const std::vector<AgentPathPredictionConfig::AbstractParamDescriptionConstPtr> &AgentPathPredictionConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<AgentPathPredictionConfig::AbstractGroupDescriptionConstPtr> &AgentPathPredictionConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const AgentPathPredictionConfigStatics *AgentPathPredictionConfig::__get_statics__()
  {
    const static AgentPathPredictionConfigStatics *statics;

    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = AgentPathPredictionConfigStatics::get_instance();

    return statics;
  }


}

#undef DYNAMIC_RECONFIGURE_FINAL

#endif // __AGENTPATHPREDICTIONRECONFIGURATOR_H__
