//#line 2 "/opt/ros/noetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"
// *********************************************************
//
// File autogenerated for the costmap_2d package
// by the dynamic_reconfigure package.
// Please do not edit.
//
// ********************************************************/

#ifndef __costmap_2d__SEMANTICPLUGINCONFIG_H__
#define __costmap_2d__SEMANTICPLUGINCONFIG_H__

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

namespace costmap_2d
{
  class SemanticPluginConfigStatics;

  class SemanticPluginConfig
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

      virtual void clamp(SemanticPluginConfig &config, const SemanticPluginConfig &max, const SemanticPluginConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const SemanticPluginConfig &config1, const SemanticPluginConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, SemanticPluginConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const SemanticPluginConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, SemanticPluginConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const SemanticPluginConfig &config) const = 0;
      virtual void getValue(const SemanticPluginConfig &config, boost::any &val) const = 0;
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
          std::string a_description, std::string a_edit_method, T SemanticPluginConfig::* a_f) :
        AbstractParamDescription(a_name, a_type, a_level, a_description, a_edit_method),
        field(a_f)
      {}

      T SemanticPluginConfig::* field;

      virtual void clamp(SemanticPluginConfig &config, const SemanticPluginConfig &max, const SemanticPluginConfig &min) const override
      {
        if (config.*field > max.*field)
          config.*field = max.*field;

        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const SemanticPluginConfig &config1, const SemanticPluginConfig &config2) const override
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, SemanticPluginConfig &config) const override
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const SemanticPluginConfig &config) const override
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, SemanticPluginConfig &config) const override
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const SemanticPluginConfig &config) const override
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const SemanticPluginConfig &config, boost::any &val) const override
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
      virtual void updateParams(boost::any &cfg, SemanticPluginConfig &top) const= 0;
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

      virtual void updateParams(boost::any &cfg, SemanticPluginConfig &top) const override
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
      std::vector<SemanticPluginConfig::AbstractGroupDescriptionConstPtr> groups;
    };

class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(SemanticPluginConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("enabled"==(*_i)->name){enabled = boost::any_cast<bool>(val);}
        if("combination_method"==(*_i)->name){combination_method = boost::any_cast<int>(val);}
      }
    }

    bool enabled;
int combination_method;

    bool state;
    std::string name;

    
}groups;



//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      bool enabled;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      int combination_method;
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
        ROS_ERROR("SemanticPluginConfig::__fromMessage__ called with an unexpected parameter.");
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
      const SemanticPluginConfig &__max__ = __getMax__();
      const SemanticPluginConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const SemanticPluginConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->calcLevel(level, config, *this);
      return level;
    }

    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const SemanticPluginConfig &__getDefault__();
    static const SemanticPluginConfig &__getMax__();
    static const SemanticPluginConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();

  private:
    static const SemanticPluginConfigStatics *__get_statics__();
  };

  template <> // Max and min are ignored for strings.
  inline void SemanticPluginConfig::ParamDescription<std::string>::clamp(SemanticPluginConfig &config, const SemanticPluginConfig &max, const SemanticPluginConfig &min) const
  {
    (void) config;
    (void) min;
    (void) max;
    return;
  }

  class SemanticPluginConfigStatics
  {
    friend class SemanticPluginConfig;

    SemanticPluginConfigStatics()
    {
SemanticPluginConfig::GroupDescription<SemanticPluginConfig::DEFAULT, SemanticPluginConfig> Default("Default", "", 0, 0, true, &SemanticPluginConfig::groups);
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.enabled = 0;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.enabled = 1;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.enabled = 1;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(SemanticPluginConfig::AbstractParamDescriptionConstPtr(new SemanticPluginConfig::ParamDescription<bool>("enabled", "bool", 0, "Whether to apply this plugin or not", "", &SemanticPluginConfig::enabled)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(SemanticPluginConfig::AbstractParamDescriptionConstPtr(new SemanticPluginConfig::ParamDescription<bool>("enabled", "bool", 0, "Whether to apply this plugin or not", "", &SemanticPluginConfig::enabled)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.combination_method = -2147483648;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.combination_method = 2147483647;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.combination_method = 1;
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(SemanticPluginConfig::AbstractParamDescriptionConstPtr(new SemanticPluginConfig::ParamDescription<int>("combination_method", "int", 0, "Method for combining two layers", "{'enum': [{'name': 'Overwrite', 'type': 'int', 'value': 0, 'srcline': 9, 'srcfile': '/home/az/arena_ws/src/arena/utils/navigation/core/costmap_2d/cfg/SemanticPlugin.cfg', 'description': 'Overwrite values', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Maximum', 'type': 'int', 'value': 1, 'srcline': 10, 'srcfile': '/home/az/arena_ws/src/arena/utils/navigation/core/costmap_2d/cfg/SemanticPlugin.cfg', 'description': 'Take the maximum of the values', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Nothing', 'type': 'int', 'value': 99, 'srcline': 11, 'srcfile': '/home/az/arena_ws/src/arena/utils/navigation/core/costmap_2d/cfg/SemanticPlugin.cfg', 'description': 'Do nothing', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'Method for combining layers enum'}", &SemanticPluginConfig::combination_method)));
//#line 291 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(SemanticPluginConfig::AbstractParamDescriptionConstPtr(new SemanticPluginConfig::ParamDescription<int>("combination_method", "int", 0, "Method for combining two layers", "{'enum': [{'name': 'Overwrite', 'type': 'int', 'value': 0, 'srcline': 9, 'srcfile': '/home/az/arena_ws/src/arena/utils/navigation/core/costmap_2d/cfg/SemanticPlugin.cfg', 'description': 'Overwrite values', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Maximum', 'type': 'int', 'value': 1, 'srcline': 10, 'srcfile': '/home/az/arena_ws/src/arena/utils/navigation/core/costmap_2d/cfg/SemanticPlugin.cfg', 'description': 'Take the maximum of the values', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'Nothing', 'type': 'int', 'value': 99, 'srcline': 11, 'srcfile': '/home/az/arena_ws/src/arena/utils/navigation/core/costmap_2d/cfg/SemanticPlugin.cfg', 'description': 'Do nothing', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'Method for combining layers enum'}", &SemanticPluginConfig::combination_method)));
//#line 246 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.convertParams();
//#line 246 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __group_descriptions__.push_back(SemanticPluginConfig::AbstractGroupDescriptionConstPtr(new SemanticPluginConfig::GroupDescription<SemanticPluginConfig::DEFAULT, SemanticPluginConfig>(Default)));
//#line 369 "/opt/ros/noetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

      for (std::vector<SemanticPluginConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__);
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__);
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__);
    }
    std::vector<SemanticPluginConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<SemanticPluginConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    SemanticPluginConfig __max__;
    SemanticPluginConfig __min__;
    SemanticPluginConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const SemanticPluginConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static SemanticPluginConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &SemanticPluginConfig::__getDescriptionMessage__()
  {
    return __get_statics__()->__description_message__;
  }

  inline const SemanticPluginConfig &SemanticPluginConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }

  inline const SemanticPluginConfig &SemanticPluginConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }

  inline const SemanticPluginConfig &SemanticPluginConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }

  inline const std::vector<SemanticPluginConfig::AbstractParamDescriptionConstPtr> &SemanticPluginConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<SemanticPluginConfig::AbstractGroupDescriptionConstPtr> &SemanticPluginConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const SemanticPluginConfigStatics *SemanticPluginConfig::__get_statics__()
  {
    const static SemanticPluginConfigStatics *statics;

    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = SemanticPluginConfigStatics::get_instance();

    return statics;
  }

//#line 9 "/home/az/arena_ws/src/arena/utils/navigation/core/costmap_2d/cfg/SemanticPlugin.cfg"
      const int SemanticPlugin_Overwrite = 0;
//#line 10 "/home/az/arena_ws/src/arena/utils/navigation/core/costmap_2d/cfg/SemanticPlugin.cfg"
      const int SemanticPlugin_Maximum = 1;
//#line 11 "/home/az/arena_ws/src/arena/utils/navigation/core/costmap_2d/cfg/SemanticPlugin.cfg"
      const int SemanticPlugin_Nothing = 99;
}

#undef DYNAMIC_RECONFIGURE_FINAL

#endif // __SEMANTICPLUGINRECONFIGURATOR_H__
