#!/usr/bin/env python3
"""
ZMQ Subscriber for Vicon Data

This script subscribes to Vicon position data published via ZMQ and displays it.
"""

import zmq
import json
import sys
import argparse
from datetime import datetime


def main():
    parser = argparse.ArgumentParser(
        description='Subscribe to Vicon data published via ZMQ'
    )
    parser.add_argument(
        '--zmq-address',
        type=str,
        default='tcp://localhost:5555',
        help='ZMQ publisher address (default: tcp://localhost:5555)'
    )
    parser.add_argument(
        '--topic-filter',
        type=str,
        default='',
        help='Topic filter prefix (empty string subscribes to all topics, default: "")'
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Output raw JSON instead of formatted messages'
    )
    parser.add_argument(
        '--quiet',
        action='store_true',
        help='Suppress output (useful for piping to other tools)'
    )
    
    args = parser.parse_args()
    
    # Create ZMQ context and socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    
    try:
        # Connect to publisher
        socket.connect(args.zmq_address)
        
        # Subscribe to topics (empty string = all topics)
        socket.setsockopt_string(zmq.SUBSCRIBE, args.topic_filter)
        
        if not args.quiet:
            print(f"Connected to ZMQ publisher at {args.zmq_address}")
            if args.topic_filter:
                print(f"Subscribed to topics matching: '{args.topic_filter}'")
            else:
                print("Subscribed to all topics")
            print("-" * 80)
        
        # Receive messages
        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)
        
        while True:
            try:
                # Poll for messages (timeout: 100ms)
                socks = dict(poller.poll(100))
                
                if socket in socks and socks[socket] == zmq.POLLIN:
                    # Receive topic name
                    topic = socket.recv_string()
                    
                    # Receive JSON data
                    data = socket.recv_string()
                    
                    # Parse JSON
                    position = json.loads(data)
                    
                    if args.json:
                        # Output raw JSON
                        print(json.dumps({
                            'topic': topic,
                            'timestamp': datetime.now().isoformat(),
                            'data': position
                        }))
                    elif not args.quiet:
                        # Output formatted message
                        print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Topic: {topic}")
                        print(f"  Subject: {position['subject_name']}")
                        print(f"  Segment: {position['segment_name']}")
                        print(f"  Translation: [{position['translation'][0]:.6f}, "
                              f"{position['translation'][1]:.6f}, "
                              f"{position['translation'][2]:.6f}]")
                        print(f"  Rotation (quaternion): [{position['rotation'][0]:.6f}, "
                              f"{position['rotation'][1]:.6f}, "
                              f"{position['rotation'][2]:.6f}, "
                              f"{position['rotation'][3]:.6f}]")
                        print(f"  Frame: {position['frame_number']}")
                        print(f"  Type: {position['translation_type']}")
                        print("-" * 80)
                    
            except json.JSONDecodeError as e:
                print(f"Error parsing JSON: {e}", file=sys.stderr)
            except KeyboardInterrupt:
                if not args.quiet:
                    print("\nShutting down...")
                break
            except Exception as e:
                print(f"Error: {e}", file=sys.stderr)
                
    except zmq.ZMQError as e:
        print(f"ZMQ Error: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        socket.close()
        context.term()


if __name__ == '__main__':
    main()

