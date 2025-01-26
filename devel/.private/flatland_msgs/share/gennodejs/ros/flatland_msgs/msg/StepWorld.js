// Auto-generated. Do not edit!

// (in-package flatland_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class StepWorld {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.required_time = null;
    }
    else {
      if (initObj.hasOwnProperty('required_time')) {
        this.required_time = initObj.required_time
      }
      else {
        this.required_time = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StepWorld
    // Serialize message field [required_time]
    bufferOffset = _serializer.float32(obj.required_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StepWorld
    let len;
    let data = new StepWorld(null);
    // Deserialize message field [required_time]
    data.required_time = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'flatland_msgs/StepWorld';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'db98664cbe4523fcf94ecdf73a95fa46';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 required_time
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StepWorld(null);
    if (msg.required_time !== undefined) {
      resolved.required_time = msg.required_time;
    }
    else {
      resolved.required_time = 0.0
    }

    return resolved;
    }
};

module.exports = StepWorld;
