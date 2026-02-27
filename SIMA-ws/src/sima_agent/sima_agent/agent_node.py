#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import ollama
from fastmcp import FastMCP
import threading
import sys

class SimaAgentNode(Node):
    def __init__(self):
        super().__init__('sima_llm_agent')
        self.get_logger().info('SIMA AI 導航 Agent 已啟動！')

        # 發布目標點給 Nav2 的 Publisher
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 初始化 FastMCP 與工具對照表
        self.mcp = FastMCP("SIMA_Nav_Tools")
        self.available_tools = {}
        self.setup_mcp_tools()

        # 開啟背景執行緒處理對話
        self.agent_thread = threading.Thread(target=self.run_agent_loop)
        self.agent_thread.daemon = True 
        self.agent_thread.start()

    def setup_mcp_tools(self):
        """將 ROS 2 導航行為註冊為 MCP 工具"""
        
        @self.mcp.tool()
        def send_nav2_goal(x: float, y: float, theta_degrees: float = 0.0) -> str:
            msg = PoseStamped()
            
            # 安全檢查：確保 ROS 2 還活著才獲取時間
            if rclpy.ok():
                msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            
            msg.pose.position.x = float(x)
            msg.pose.position.y = float(y)
            msg.pose.position.z = 0.0

            theta_rad = math.radians(theta_degrees)
            msg.pose.orientation.z = math.sin(theta_rad / 2.0)
            msg.pose.orientation.w = math.cos(theta_rad / 2.0)

            self.goal_pub.publish(msg)
            
            log_msg = f"已發送導航目標至座標: X={x}, Y={y}, 朝向={theta_degrees}度"
            if rclpy.ok():
                self.get_logger().info(log_msg)
            return log_msg

        # 將裝飾好的函數存入字典，讓後面的大腦能呼叫到
        self.available_tools['send_nav2_goal'] = send_nav2_goal

    def run_agent_loop(self):
        """背景執行的：對話與呼叫大腦的迴圈"""
        tools_schema = [{
            'type': 'function',
            'function': {
                'name': 'send_nav2_goal',
                'description': '發送導航目標點給機器人。',
                'parameters': {
                    'type': 'object',
                    'properties': {
                        'x': {'type': 'number'},
                        'y': {'type': 'number'},
                        'theta_degrees': {'type': 'number'}
                    },
                    # 注意：我們不把 theta_degrees 設為 required，這樣 LLM 沒給也沒關係
                    'required': ['x', 'y']
                }
            }
        }]

        messages = [{'role': 'system', 'content': '你是 SIMA 機器人的導航指揮官。你可以呼叫工具發送座標。'}]
        print("\n🤖 SIMA Agent 已就緒！可以開始下達指令 (輸入 exit 離開)")
        
        while rclpy.ok():
            try:
                user_input = input("\n指揮官: ")
                if user_input.lower() in ['exit', 'quit']:
                    print('\n👋 收到退出指令，準備關閉 Agent...')
                    # 觸發主執行緒關閉
                    raise KeyboardInterrupt

                if not user_input.strip():
                    continue

                messages.append({'role': 'user', 'content': user_input})
                print("🧠 思考中...")
                
                response = ollama.chat(
                    model='qwen2.5:1.5b',
                    messages=messages,
                    tools=tools_schema,
                    keep_alive=0 
                )

                # debug: print out original response
                print("\n" + "="*50)
                print("🔍 【Ollama 原始回傳資料 (Raw Response)】:")
                # 為了讓字典印出來比較好看，我們可以用這招
                import json
                # 如果 response 裡面有不能轉 json 的物件，這行可能會報錯，
                # 但 ollama 的 Python 庫通常回傳乾淨的 dict。
                # 如果報錯，可以改成最簡單的: print(response)
                try:
                    print(json.dumps(response, indent=2, ensure_ascii=False))
                except:
                    print(response)
                print("="*50 + "\n")
                
                message = response['message']
                
                if message.get('tool_calls'):
                    for tool in message['tool_calls']:
                        func_name = tool['function']['name']
                        
                        if func_name in self.available_tools:
                            args = tool['function']['arguments']
                            print(f"🛠️  調用工具 [{func_name}]，參數: {args}")
                            
                            # 執行 ROS 2 發布動作
                            result = self.available_tools[func_name](**args) 
                            
                            messages.append(message)
                            messages.append({'role': 'tool', 'content': result})
                            
                    # 總結報告
                    final_response = ollama.chat(model='qwen2.5:1.5b', messages=messages, keep_alive=0)
                    print(f"🤖 SIMA: {final_response['message']['content']}")
                    messages.append(final_response['message'])
                else:
                    print(f"🤖 SIMA: {message['content']}")
                    messages.append(message)
                    
            except EOFError:
                break # 處理 Ctrl+D
            except KeyboardInterrupt:
                break # 處理 Ctrl+C
            except Exception as e:
                # 只有在 ROS 2 還活著的時候才印 Error，避免關閉時噴紅字
                if rclpy.ok():
                    self.get_logger().error(f"Agent 發生錯誤: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SimaAgentNode()
    
    try:
        # 主執行緒專心監聽 ROS 2 網路
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕捉 Ctrl+C，什麼都不印，優雅關閉
        pass
    finally:
        # 清理戰場
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()