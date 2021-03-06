/* AUTO-GENERATED FILE.  DO NOT MODIFY.
 *
 * This class was automatically generated by the
 * java mavlink generator tool. It should not be modified by hand.
 */

// MESSAGE CAMERA_CAPTURE_STATUS PACKING
package com.MAVLink.common;
import com.MAVLink.MAVLinkPacket;
import com.MAVLink.Messages.MAVLinkMessage;
import com.MAVLink.Messages.MAVLinkPayload;
        
/**
* WIP: Information about the status of a capture
*/
public class msg_camera_capture_status extends MAVLinkMessage{

    public static final int MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS = 262;
    public static final int MAVLINK_MSG_LENGTH = 23;
    private static final long serialVersionUID = MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS;


      
    /**
    * Timestamp (milliseconds since system boot)
    */
    public long time_boot_ms;
      
    /**
    * Image capture interval in seconds
    */
    public float image_interval;
      
    /**
    * Video frame rate in Hz
    */
    public float video_framerate;
      
    /**
    * Image resolution in pixels horizontal
    */
    public int image_resolution_h;
      
    /**
    * Image resolution in pixels vertical
    */
    public int image_resolution_v;
      
    /**
    * Video resolution in pixels horizontal
    */
    public int video_resolution_h;
      
    /**
    * Video resolution in pixels vertical
    */
    public int video_resolution_v;
      
    /**
    * Camera ID if there are multiple
    */
    public short camera_id;
      
    /**
    * Current status of image capturing (0: not running, 1: interval capture in progress)
    */
    public short image_status;
      
    /**
    * Current status of video capturing (0: not running, 1: capture in progress)
    */
    public short video_status;
    

    /**
    * Generates the payload for a mavlink message for a message of this type
    * @return
    */
    public MAVLinkPacket pack(){
        MAVLinkPacket packet = new MAVLinkPacket(MAVLINK_MSG_LENGTH);
        packet.sysid = 255;
        packet.compid = 190;
        packet.msgid = MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS;
              
        packet.payload.putUnsignedInt(time_boot_ms);
              
        packet.payload.putFloat(image_interval);
              
        packet.payload.putFloat(video_framerate);
              
        packet.payload.putUnsignedShort(image_resolution_h);
              
        packet.payload.putUnsignedShort(image_resolution_v);
              
        packet.payload.putUnsignedShort(video_resolution_h);
              
        packet.payload.putUnsignedShort(video_resolution_v);
              
        packet.payload.putUnsignedByte(camera_id);
              
        packet.payload.putUnsignedByte(image_status);
              
        packet.payload.putUnsignedByte(video_status);
        
        return packet;
    }

    /**
    * Decode a camera_capture_status message into this class fields
    *
    * @param payload The message to decode
    */
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
              
        this.time_boot_ms = payload.getUnsignedInt();
              
        this.image_interval = payload.getFloat();
              
        this.video_framerate = payload.getFloat();
              
        this.image_resolution_h = payload.getUnsignedShort();
              
        this.image_resolution_v = payload.getUnsignedShort();
              
        this.video_resolution_h = payload.getUnsignedShort();
              
        this.video_resolution_v = payload.getUnsignedShort();
              
        this.camera_id = payload.getUnsignedByte();
              
        this.image_status = payload.getUnsignedByte();
              
        this.video_status = payload.getUnsignedByte();
        
    }

    /**
    * Constructor for a new message, just initializes the msgid
    */
    public msg_camera_capture_status(){
        msgid = MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS;
    }

    /**
    * Constructor for a new message, initializes the message with the payload
    * from a mavlink packet
    *
    */
    public msg_camera_capture_status(MAVLinkPacket mavLinkPacket){
        this.sysid = mavLinkPacket.sysid;
        this.compid = mavLinkPacket.compid;
        this.msgid = MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS;
        unpack(mavLinkPacket.payload);        
    }

                        
    /**
    * Returns a string with the MSG name and data
    */
    public String toString(){
        return "MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS -"+" time_boot_ms:"+time_boot_ms+" image_interval:"+image_interval+" video_framerate:"+video_framerate+" image_resolution_h:"+image_resolution_h+" image_resolution_v:"+image_resolution_v+" video_resolution_h:"+video_resolution_h+" video_resolution_v:"+video_resolution_v+" camera_id:"+camera_id+" image_status:"+image_status+" video_status:"+video_status+"";
    }
}
        