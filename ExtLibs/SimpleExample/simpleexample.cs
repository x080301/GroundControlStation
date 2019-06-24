using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;




namespace SimpleExample
{
    public partial class simpleexample : Form
    {
        MAVLink.MavlinkParse mavlink = new MAVLink.MavlinkParse();//MAVLINK解析包函数，通过这个函数从串口得到一帧的数据
            bool armed = false;
        // locking to prevent multiple reads on serial port
        object readlock = new object();
        // our target sysid
        byte sysid=1;
        // our target compid
        byte compid=1;

        public simpleexample()
        {
            InitializeComponent();
        }
        

        private void but_connect_Click(object sender, EventArgs e)
        {
            // if the port is open close it
            if (serialPort1.IsOpen)
            {
                serialPort1.Close();
                return;
            }

            // set the comport options
            serialPort1.PortName = CMB_comport.Text; //打开串口
            serialPort1.BaudRate = int.Parse(cmb_baudrate.Text);

            // open the comport
            serialPort1.Open();

            // set timeout to 2 seconds
            serialPort1.ReadTimeout = 2000;

            //启动单独的线程
            BackgroundWorker bgw = new BackgroundWorker();

            //开始接收包,并作相应的处理
            bgw.DoWork += bgw_DoWork;
            //连接质量判断
            bgw.DoWork += if_lost_connection;
            //图像识别模块
            //bgw.DoWork += IIModule;
            //控制模块
            //bgw.DoWork += flight_controler;
            //测试信息发送模块
            bgw.DoWork += TestSend1;
            bgw.DoWork += TestSend2;

            bgw.RunWorkerAsync();
        }

        //测试信息发送模块 
        static DateTime Time4TestSend = DateTime.Now.AddSeconds(1);
        void TestSend1(object sender, DoWorkEventArgs e)//sender与e为开启单独线程所用
        {
            while (serialPort1.IsOpen)
            {
                if (DateTime.Now > Time4TestSend)
                {
                    MAVLink.mavlink_command_long_t req1 = new MAVLink.mavlink_command_long_t();
                    req1.target_system = sysid;
                    req1.target_component = compid;
                    req1.command = (ushort)2;
                    byte[] packet1 = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, req1);
                    serialPort1.Write(packet1, 0, packet1.Length);

                    Time4TestSend = DateTime.Now.AddSeconds(1);
                }
            }
        }
        void TestSend2(object sender, DoWorkEventArgs e)//sender与e为开启单独线程所用
        {
            while (serialPort1.IsOpen)
            {
                if (DateTime.Now > Time4TestSend)
                {
                    MAVLink.mavlink_heartbeat_t req2 = new MAVLink.mavlink_heartbeat_t();
                    byte[] packet2 = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, req2);
                    serialPort1.Write(packet2, 0, packet2.Length);

                    Time4TestSend = DateTime.Now.AddSeconds(1);
                }
            }
        }


        //图像识别模块
        static bool II_arrived = false;//是否识别到标记/是否抵达目的地
        static int x=0;//所识别到标记的中心点，在图中的x坐标
        static int y=0;//所识别到标记的中心点，在图中的y坐标
        static bool II_finished = false;
        void IIModule(object sender, DoWorkEventArgs e)//sender与e为开启单独线程所用
        {
            
        }
        //处理模块
        void flight_controler(object sender, DoWorkEventArgs e)
        {
            while(serialPort1.IsOpen)
            {
                if(!II_finished)
                {
                    continue;
                }
                II_finished = false;

                //执行相关的命令处理
                ushort cmd4sender=0;

                //命令发送
                flight_control_commond_sender(cmd4sender);

            }
        }
        //指令发送模块
        void flight_control_commond_sender(ushort cmd4sender)
        {
            MAVLink.mavlink_command_long_t req = new MAVLink.mavlink_command_long_t();

            req.target_system = sysid;
            req.target_component = compid;

            req.command = cmd4sender;

            /*
            req.param1 = p1;
            req.param2 = p2;
            req.param3 = p3;
            req.param4 = p4;
            req.param5 = p5;
            req.param6 = p6;
            req.param7 = p7;
            */

            byte[] packet = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, req);

            serialPort1.Write(packet, 0, packet.Length);
            /*回执包未准备好
            try
            {//200ms内，收信息。若收到成功动作的信息，结束收取。若未收到其他类型信息，输出而后继续收。若200ms后依然没收到，那么直接结束。
                var ack = readsomedata<MAVLink.mavlink_command_ack_t>(sysid, compid);
                //if (ack.result == (byte)MAVLink.MAV_RESULT.ACCEPTED)
            }
            catch
            {
            }
            */
        }



        //连接质量考量<-心跳包改为10Hz
        static int Counter4heartbeat = 0;
        static DateTime Time4heartbeat = DateTime.Now.AddSeconds(1);
        static int connect_statue = 0;//0连接正常，1差，2失联
        void if_lost_connection(object sender, DoWorkEventArgs e)
        {
            while(serialPort1.IsOpen)
            {
                if (DateTime.Now < Time4heartbeat)
                    continue;

                if (Counter4heartbeat == 0)
                {
                    if (connect_statue != 2)
                    {
                        connect_statue = 2;
                        Console.WriteLine("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
                        Console.WriteLine("失去连接");
                        Console.WriteLine("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
                    }
                }
                else if (Counter4heartbeat <= 6)
                {
                    if (connect_statue != 1)
                    {
                        connect_statue = 1;
                        Console.WriteLine("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
                        Console.WriteLine("连接质量差");
                        Console.WriteLine("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
                    }
                }
                else
                {
                    if (connect_statue != 0)
                    {
                        connect_statue = 0;
                        Console.WriteLine("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
                        Console.WriteLine("连接恢复");
                        Console.WriteLine("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
                    }
                }

                Time4heartbeat = DateTime.Now.AddSeconds(1);
                Counter4heartbeat = 0;
            }
        }

        void bgw_DoWork(object sender, DoWorkEventArgs e)
        {
            while (serialPort1.IsOpen)
            {
                try
                {
                    MAVLink.MAVLinkMessage packet;
                    lock (readlock)
                    {
                        // read any valid packet from the port
                        packet = mavlink.ReadPacket(serialPort1.BaseStream);
                        
                        // check its valid
                        if (packet == null || packet.data == null)
                            continue;
                    }



                    // check to see if its a hb packet from the comport
                    if (packet.data.GetType() == typeof(MAVLink.mavlink_heartbeat_t))
                    {
                        Counter4heartbeat++;
                        var hb = (MAVLink.mavlink_heartbeat_t)packet.data;

                        // save the sysid and compid of the seen MAV
                        sysid = packet.sysid;
                        compid = packet.compid;

                        // request streams at 2 hz
                        mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.REQUEST_DATA_STREAM,
                            new MAVLink.mavlink_request_data_stream_t()
                            {
                                req_message_rate = 2,
                                req_stream_id = (byte)MAVLink.MAV_DATA_STREAM.ALL,
                                start_stop = 1,
                                target_component = compid,
                                target_system = sysid
                            });
                        Console.WriteLine("心跳包");
                    }

                    // from here we should check the the message is addressed to us
                    if (sysid != packet.sysid || compid != packet.compid)
                        continue;
                    
                    if(packet.msgid==(byte)MAVLink.MAVLINK_MSG_ID.COMMAND_LONG)
                    {
                        var att = (MAVLink.mavlink_command_long_t)packet.data;
                        int cmd = (int)att.command;
                        if (cmd == 1)
                            Console.WriteLine("心跳包回执");
                        else if(cmd==2)
                            Console.WriteLine("长消息回执");
                        else
                            Console.WriteLine("未知错误");
                    }
                    if (packet.msgid == (byte)MAVLink.MAVLINK_MSG_ID.ATTITUDE)
                    //or
                    //if (packet.data.GetType() == typeof(MAVLink.mavlink_attitude_t))
                    {
                        var att = (MAVLink.mavlink_attitude_t)packet.data;

                        Console.WriteLine(att.pitch*57.2958 + " " + att.roll*57.2958);
                    }
                }
                catch
                {
                }

                System.Threading.Thread.Sleep(1);
            }
        }

        T readsomedata<T>(byte sysid,byte compid,int timeout = 2000)
        {
            DateTime deadline = DateTime.Now.AddMilliseconds(timeout);

            lock (readlock)
            {
                // read the current buffered bytes
                while (DateTime.Now < deadline)
                {
                    var packet = mavlink.ReadPacket(serialPort1.BaseStream);

                    // check its not null, and its addressed to us
                    if (packet == null || sysid != packet.sysid || compid != packet.compid)
                        continue;

                    Console.WriteLine(packet);

                    if (packet.data.GetType() == typeof (T))
                    {
                        return (T) packet.data;
                    }
                }
            }

            throw new Exception("No packet match found");
        }

        private void armdisarm()
        {
            MAVLink.mavlink_command_long_t req = new MAVLink.mavlink_command_long_t();

            req.target_system = 1;
            req.target_component = 1;

            req.command = (ushort)MAVLink.MAV_CMD.COMPONENT_ARM_DISARM;

            req.param1 = armed ? 0 : 1;
            armed = !armed;
            /*
            req.param2 = p2;
            req.param3 = p3;
            req.param4 = p4;
            req.param5 = p5;
            req.param6 = p6;
            req.param7 = p7;
            */

            byte[] packet = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, req);

            serialPort1.Write(packet, 0, packet.Length);

            try
            {//readsomedata<MAVLink.mavlink_command_ack_t>：200ms内，收信息。若收到成功动作的信息，结束收取。若未收到其他类型信息，输出而后继续收。若200ms后依然没收到，那么直接结束。
                var ack = readsomedata<MAVLink.mavlink_command_ack_t>(sysid, compid);
                if (ack.result == (byte)MAVLink.MAV_RESULT.ACCEPTED)
                {

                }
            }
            catch
            {
            }
        }
        private void but_armdisarm_Click(object sender, EventArgs e)
        {
            armdisarm();
        }

        private void CMB_comport_Click(object sender, EventArgs e)
        {
            CMB_comport.DataSource = SerialPort.GetPortNames();
        }

        private void CMB_comport_SelectedIndexChanged(object sender, EventArgs e)
        {

        }
    }
}
