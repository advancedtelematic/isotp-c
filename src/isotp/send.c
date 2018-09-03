#include <isotp/send.h>
#include <isotp/protocol.h>
#include <bitfield/bitfield.h>
#include <string.h>

#define PCI_NIBBLE_INDEX 0
#define PAYLOAD_LENGTH_NIBBLE_INDEX 1
#define PAYLOAD_BYTE_INDEX 1

void isotp_complete_send(IsoTpShims* shims, IsoTpMessage* message,
        bool status, IsoTpMessageSentHandler callback) {
    (void) shims;
    if(callback != NULL) {
        callback(message, status);
    }
}

IsoTpSendHandle isotp_send_single_frame(IsoTpShims* shims, IsoTpMessage* message,
        IsoTpMessageSentHandler callback) {
    IsoTpSendHandle handle = {
        success: false,
        completed: true
    };

    uint8_t can_data[CAN_MESSAGE_BYTE_SIZE] = {0};
    if(!set_nibble(PCI_NIBBLE_INDEX, PCI_SINGLE, can_data, sizeof(can_data))) {
	if(shims->log)
            shims->log("Unable to set PCI in CAN data");
        return handle;
    }

    if(!set_nibble(PAYLOAD_LENGTH_NIBBLE_INDEX, message->size, can_data,
                sizeof(can_data))) {
	if(shims->log)
            shims->log("Unable to set payload length in CAN data");
        return handle;
    }

    if(message->size > 0) {
        memcpy(&can_data[1], message->payload, message->size);
    }

    handle.success = shims->send_can_message(message->arbitration_id, can_data,
            shims->frame_padding ? 8 : 1 + message->size, shims->private_data);
    isotp_complete_send(shims, message, handle.success, callback);
    return handle;
}

IsoTpSendHandle isotp_send_multi_frame(IsoTpShims* shims, IsoTpMessage* message,
        IsoTpMessageSentHandler callback) {
    IsoTpSendHandle handle = {
        success: false,
        completed: true
    };
    uint8_t can_data[CAN_MESSAGE_BYTE_SIZE] = {0};
    if(!set_nibble(PCI_NIBBLE_INDEX, PCI_FIRST_FRAME, can_data, sizeof(can_data))) {
	if(shims->log)
            shims->log("Unable to set PCI in CAN data");
        return handle;
    }

    if(!set_nibble(PAYLOAD_LENGTH_NIBBLE_INDEX, message->size >> 8, can_data,
                sizeof(can_data))) {
	if(shims->log)
            shims->log("Unable to set payload length in CAN data");
        return handle;
    }

    /* One we got here message->size > 7 */

    can_data[1] = message->size & 0xFF;

    memcpy(&can_data[2], message->payload, 6);

    if(!shims->send_can_message(message->arbitration_id, can_data, 8, shims->private_data)) {
	    if(shims->log)
		    shims->log("Error sending CAN frame");
	    return handle;
    }

    handle.message = message;
    handle.payload_index = 6;
    handle.seqn = 1;
    handle.completed = false;
    handle.to_send = 0;
    handle.message_sent_callback = callback;
    handle.sending_arbitration_id = message->arbitration_id;

    return handle;
}

IsoTpMessage isotp_new_send_message(const uint16_t arbitration_id, const uint8_t payload[], uint16_t size) {
    IsoTpMessage message = {
        arbitration_id: arbitration_id,
        size: size
    };

    memcpy(message.payload, payload, size);
    return message;
}
IsoTpSendHandle isotp_send(IsoTpShims* shims, IsoTpMessage* message, IsoTpMessageSentHandler callback) {
    if(message->size < 8) {
        return isotp_send_single_frame(shims, message, callback);
    } else {
        return isotp_send_multi_frame(shims, message, callback);
    }
}

bool isotp_receive_flowcontrol(IsoTpShims* shims, IsoTpSendHandle* handle,
        const uint16_t arbitration_id, const uint8_t data[],
        const uint8_t size) {

    if(size < 3 || get_nibble(data, 3, PCI_NIBBLE_INDEX) != PCI_FLOW_CONTROL_FRAME) {
        if(shims->log) {
            shims->log("Incorrect size (%d while %d expected) or frame type (%1x while %1x expected)",
                    size, 3, get_nibble(data, 3, PCI_NIBBLE_INDEX), PCI_FLOW_CONTROL_FRAME);
        }
        return false;
    }

    switch(get_nibble(data, 3, FLOW_CONTROL_NIBBLE_INDEX)) {
	    case PCI_FLOW_STATUS_CONTINUE:
		    if(data[1] == 0)
			    handle->to_send = -1;
		    else
			    handle->to_send = data[1];

		    /* TODO separation time should not be ignored */
		    break;
	    case PCI_FLOW_STATUS_WAIT:
		    handle->to_send = 0;
		    break;
	    case PCI_FLOW_STATUS_OVERFLOW:
		    handle->completed = true;
		    handle->success = false;
		    if(shims->log) {
			    shims->log("ISO TP overflow");
		    }
		    return true;
	    default:
		    if(shims->log) {
			    shims->log("Unexpected flow control message type: %x", get_nibble(data, 3, FLOW_CONTROL_NIBBLE_INDEX));
		    }
		    return false;
    }
    return true;
}

bool isotp_continue_send(IsoTpShims* shims, IsoTpSendHandle* handle) {
	uint8_t frame_len;
	bool is_last = false;
	uint8_t can_data[CAN_MESSAGE_BYTE_SIZE] = {0};

	if(handle->to_send == 0)
		return true;

	if(!set_nibble(PCI_NIBBLE_INDEX, PCI_CONSECUTIVE_FRAME, can_data, sizeof(can_data))) {
		if(shims->log)
			shims->log("Unable to set PCI in CAN data");
		handle->completed = true;
		handle->success = false;
		isotp_complete_send(shims, handle->message, false, handle->message_sent_callback);
		return false;
	}

	if(!set_nibble(SEQUENCE_NUMBER_NIBBLE_INDEX, handle->seqn++, can_data, sizeof(can_data))) {
		if(shims->log)
			shims->log("Unable to set payload length in CAN data");
		handle->completed = true;
		handle->success = false;
		isotp_complete_send(shims, handle->message, false, handle->message_sent_callback);
		return false;
	}

	if(handle->payload_index + 7 >= handle->message->size) {
		frame_len = handle->message->size - handle->payload_index;
		is_last = true;
	} else {
		frame_len = 7;
		is_last = false;
	}

	memcpy(&can_data[1], &handle->message->payload[handle->payload_index], frame_len);

	if(!shims->send_can_message(handle->sending_arbitration_id, can_data, frame_len+1, shims->private_data)) {
		if(shims->log)
			shims->log("Error sending CAN frame");
		handle->completed = true;
		handle->success = false;
		isotp_complete_send(shims, handle->message, false, handle->message_sent_callback);
		return false;
	}

	if(is_last) {
		handle->completed = true;
		handle->success = true;
		isotp_complete_send(shims, handle->message, true, handle->message_sent_callback);
	} else {
		if(handle->to_send > 0) {
			handle->to_send -= 1;
		}
		handle->payload_index += 7;
	}

	return true;
}

