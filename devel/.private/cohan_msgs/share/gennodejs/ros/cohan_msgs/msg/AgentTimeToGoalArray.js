// Auto-generated. Do not edit!

// (in-package cohan_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let AgentTimeToGoal = require('./AgentTimeToGoal.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class AgentTimeToGoalArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.times_to_goal = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('times_to_goal')) {
        this.times_to_goal = initObj.times_to_goal
      }
      else {
        this.times_to_goal = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AgentTimeToGoalArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [times_to_goal]
    // Serialize the length for message field [times_to_goal]
    bufferOffset = _serializer.uint32(obj.times_to_goal.length, buffer, bufferOffset);
    obj.times_to_goal.forEach((val) => {
      bufferOffset = AgentTimeToGoal.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AgentTimeToGoalArray
    let len;
    let data = new AgentTimeToGoalArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [times_to_goal]
    // Deserialize array length for message field [times_to_goal]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.times_to_goal = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.times_to_goal[i] = AgentTimeToGoal.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.times_to_goal.forEach((val) => {
      length += AgentTimeToGoal.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cohan_msgs/AgentTimeToGoalArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e2409fda334715bb8c8e0025d4e039a0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header             header
    cohan_msgs/AgentTimeToGoal[] times_to_goal
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: cohan_msgs/AgentTimeToGoal
    std_msgs/Header header
    uint64          id
    int8            type
    duration        time_to_goal
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AgentTimeToGoalArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.times_to_goal !== undefined) {
      resolved.times_to_goal = new Array(msg.times_to_goal.length);
      for (let i = 0; i < resolved.times_to_goal.length; ++i) {
        resolved.times_to_goal[i] = AgentTimeToGoal.Resolve(msg.times_to_goal[i]);
      }
    }
    else {
      resolved.times_to_goal = []
    }

    return resolved;
    }
};

module.exports = AgentTimeToGoalArray;
