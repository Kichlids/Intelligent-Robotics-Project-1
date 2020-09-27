// Auto-generated. Do not edit!

// (in-package robot_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class keyboard {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.is_teleop = null;
      this.command = null;
    }
    else {
      if (initObj.hasOwnProperty('is_teleop')) {
        this.is_teleop = initObj.is_teleop
      }
      else {
        this.is_teleop = false;
      }
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type keyboard
    // Serialize message field [is_teleop]
    bufferOffset = _serializer.bool(obj.is_teleop, buffer, bufferOffset);
    // Serialize message field [command]
    bufferOffset = _serializer.string(obj.command, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type keyboard
    let len;
    let data = new keyboard(null);
    // Deserialize message field [is_teleop]
    data.is_teleop = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [command]
    data.command = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.command.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_msgs/keyboard';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c9b3c6bb662db54412007781db0793c9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool is_teleop
    string command
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new keyboard(null);
    if (msg.is_teleop !== undefined) {
      resolved.is_teleop = msg.is_teleop;
    }
    else {
      resolved.is_teleop = false
    }

    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = ''
    }

    return resolved;
    }
};

module.exports = keyboard;
