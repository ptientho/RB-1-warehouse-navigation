const { createApp } = Vue

createApp({

    data() {
    
        return {
            // ROS connection
            ros: null,
            rosbridge_address: '', 
            
            // button status
            connected: false,
            status: 'Connect',
            // content
            main_title1: 'Manual Control',
            main_title2: 'Autonomy BT',
            main_title3: 'Service Status',
            // velocity pub
            velPub: null,

            // select robot
            selectedEnvi: '',
            velTopic: '',
            liftUpTopic: '',
            liftDownTopic: '',
            btService: '',
            clickedPoint: '',
            serviceIdleState: true,
            serviceResponse1: '',
            serviceResponse2: '',
            serviceResponse3: '',
            serviceResponse4: '',
            serviceResponseFull: '',

            // behavior tree parameter
            btXML_1: '',
            btXML_2: '',
            btXML_3: '',
            btXML_4: '',
            btXML_full: '',
            btXML_home: '',
            
            // elevator control
            isElevatorUp: false,

            // drag zone style
            dragCircleStyle: {
            margin: '0px',
            top: '0px',
            left: '0px',
            display: 'none',
            width: '75px',
            height: '75px',
            },
            //dragging data
            dragging: false,
            x: 'no',
            y: 'no',
            // joystick valules
            joystick: {
                vertical: 0,
                horizontal: 0,
            },

            // available topics and services
            topics: [],
            services: [],
            // service checker
            shelfDetectSrv: '',
            shelfAttachSrv: '',
            shelfDetachSrv: '',
            backUpSrv: '',
        };
    },
    methods: {
        /* connect ro Rosbridge */
        connect(){
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            });
            console.log(`Rosbridge address set as: ${this.rosbridge_address}`);
            // Connection callback
            this.ros.on('connection', () => {
            
                console.log('Connection established!');
                this.connected = true;
                this.velPub = setInterval(this.sendVelocity, 100);
            });

            this.ros.on('close', () => {
            
                console.log('Connection closed!');
                this.connected = false;
                this.resetFlags();
            });

            this.ros.on('error', (error) => {
                console.log('Something went wrong when trying to connect')
                console.log(error)
            });
        },
        /* disconnect */
        disconnect(){
            this.ros.close();
        },
        /* reset status of bt */
        resetFlags(){
            this.serviceIdleState = true;
            this.serviceResponse1 = '';
            this.serviceResponse2 = '';
            this.serviceResponse3 = '';
            this.serviceResponse4 = '';
            this.serviceResponseFull = '';
        },
        /* get ros address from user */
        get_address(e){
            this.rosbridge_address = e.target.value;
        },
        /* toggle connection button */
        toggle(){
            if (this.rosbridge_address !== ''){
                
                if (this.connected === false) {
                    this.status = 'Disconnect';
                    this.connect();
                }else{
                    this.status = 'Connect';
                    this.disconnect();
                }
            }
        },

        onSelectEnvi(e){
            this.selectedEnvi = e.target.value;
            
            // set robot environment parameters here!
            if (this.selectedEnvi === 'Simulated Robot') {
                console.log(`Robot Environment: ${this.selectedEnvi}`);
                this.velTopic = '/diffbot_base_controller/cmd_vel_unstamped';
                this.liftUpTopic = '/elevator_up';
                this.liftDownTopic = '/elevator_down';
                this.clickedPoint = '/clicked_point';
                this.btXML_1 = 'bt_test_shelf_detect.xml';
                this.btXML_2 = 'bt_test_go_to_pose.xml';
                this.btXML_3 = 'bt_test_shelf_attach.xml';
                this.btXML_4 = 'bt_test_shelf_detach.xml';
                this.btXML_full = 'bt.xml';
                this.btXML_home = 'bt_test_home.xml';
                this.btService = '/rb1_autonomy_server';
                this.shelfDetectSrv = '/go_to_shelf';
                this.shelfAttachSrv = '/attach_shelf';
                this.shelfDetachSrv = '/detach_shelf';
                this.backUpSrv = '/backup';
                
            } else if(this.selectedEnvi === 'Real Robot'){
                console.log(`Robot Environment: ${this.selectedEnvi}`);
                this.velTopic = '/cmd_vel';
                this.liftUpTopic = '/elevator_up';
                this.liftDownTopic = '/elevator_down';
                this.clickedPoint = '/clicked_point';
                this.btXML_1 = 'bt_test_shelf_detect_real.xml';
                this.btXML_2 = 'bt_test_go_to_pose_w_shelf_detect_real.xml';
                this.btXML_3 = 'bt_test_shelf_attach_real.xml';
                this.btXML_4 = 'bt_test_shelf_detach.xml';
                this.btXML_full = 'bt_real.xml';
                this.btXML_home = 'bt_test_home.xml';
                this.btService = '/rb1_autonomy_server';
                this.shelfDetectSrv = '/go_to_shelf_real';
                this.shelfAttachSrv = '/attach_shelf';
                this.shelfDetachSrv = '/detach_shelf';
                this.backUpSrv = '/backup';
            }
        },        
        /* send velocity command */
        startDrag() {
            console.log('start dragging');
            this.dragging = true
            this.x = this.y = 0
        },
        stopDrag() {
            console.log('stop dragging');
            this.dragging = false
            this.x = this.y = 'no'
            this.dragCircleStyle.display = 'none'
            this.resetJoystickVals()
        },
        doDrag(event) {
            if (this.dragging) {
                console.log('dragging');
                this.x = event.offsetX
                this.y = event.offsetY
                let ref = document.getElementById('dragstartzone')
                this.dragCircleStyle.display = 'inline-block'

                let minTop = ref.offsetTop - parseInt(this.dragCircleStyle.height) / 2
                let maxTop = minTop + 200
                let top = this.y + minTop
                this.dragCircleStyle.top = `${top}px`

                let minLeft = ref.offsetLeft - parseInt(this.dragCircleStyle.width) / 2
                let maxLeft = minLeft + 200
                let left = this.x + minLeft
                this.dragCircleStyle.left = `${left}px`

                this.setJoystickVals()
            }
        },
        setJoystickVals() {
            this.joystick.vertical = -1 * ((this.y / 200) - 0.5)
            this.joystick.horizontal = +1 * ((this.x / 200) - 0.5)
        },
        resetJoystickVals() {
            this.joystick.vertical = 0
            this.joystick.horizontal = 0
        },
        sendVelocity(){
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: this.velTopic,
                messageType: 'geometry_msgs/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: this.joystick.vertical, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: this.joystick.horizontal, },
            })
            topic.publish(message)
        
        },
        request_bt1(){
            
            let client = new ROSLIB.Service({
                ros: this.ros,
                name: this.btService,
                serviceType: 'rb1_autonomy_msg/TickBT',
            })
            console.log('sending server request to ' + client.name);
            
            let request = new ROSLIB.ServiceRequest({
                bt_xml_file : this.btXML_1
            });

            this.serviceIdleState = false;
            client.callService(request, (result) => {
                console.log('Result for service call on ' + client.name + ': Fould cart? --->' + result.bt_status);
                this.serviceIdleState = true;
                this.serviceResponse1 = result.bt_status ? 'cart is found' : 'cart is not found';
            }, (error) => {
                console.error(error);
                this.serviceIdleState = true;                
            });
            
        },
        request_bt2(){
            
            let client = new ROSLIB.Service({
                ros: this.ros,
                name: this.btService,
                serviceType: 'rb1_autonomy_msg/TickBT'
            })
            console.log('sending server request to ' + client.name);
            
            let request = new ROSLIB.ServiceRequest({
                bt_xml_file : this.btXML_2
            });

            this.serviceIdleState = false;
            client.callService(request, (result) => {
                console.log('Result for service call on ' + client.name + ': ' + result.bt_status);
                this.serviceIdleState = true;
                this.serviceResponse2 = result.bt_status ? 'cart position reached' : 'cart position not reached';
            }, (error) => {
                console.error(error);
                this.serviceIdleState = true;
            });
            
        },
        request_bt3(){
            let client = new ROSLIB.Service({
                ros: this.ros,
                name: this.btService,
                serviceType: 'rb1_autonomy_msg/TickBT'
            })
            console.log('sending server request to ' + client.name);
            
            let request = new ROSLIB.ServiceRequest({
                bt_xml_file : this.btXML_3
            });

            this.serviceIdleState = false;
            client.callService(request, (result) => {
                console.log('Result for service call on ' + client.name + ': ' + result.bt_status);
                this.serviceIdleState = true;
                this.serviceResponse3 = result.bt_status ? 'cart attached' : 'cart not attached';
            }, (error) => {
                console.error(error);
                this.serviceIdleState = true;
            });

        },
        request_bt4(){
            let client = new ROSLIB.Service({
                ros: this.ros,
                name: this.btService,
                serviceType: 'rb1_autonomy_msg/TickBT'
            })
            console.log('sending server request to ' + client.name);
            
            let request = new ROSLIB.ServiceRequest({
                bt_xml_file : this.btXML_4
            });

            this.serviceIdleState = false;
            client.callService(request, (result) => {
                console.log('Result for service call on ' + client.name + ': ' + result.bt_status);
                this.serviceIdleState = true;
                this.serviceResponse4 = result.bt_status ? 'cart detached' : 'cart not detached';
            }, (error) => {
                console.error(error);
                this.serviceIdleState = true;
            });

        },
        request_bt_full(){
            let client = new ROSLIB.Service({
                ros: this.ros,
                name: this.btService,
                serviceType: 'rb1_autonomy_msg/TickBT'
            })
            console.log('sending server request to ' + client.name);
            
            let request = new ROSLIB.ServiceRequest({
                bt_xml_file : this.btXML_full
            });

            this.serviceIdleState = false;
            client.callService(request, (result) => {
                console.log('Result for service call on ' + client.name + ': ' + result.bt_status);
                this.serviceIdleState = true;
                this.serviceResponseFull = result.bt_status ? 'Succeeded' : 'Failure';
            }, (error) => {
                console.error(error);
                this.serviceIdleState = true;
            });
        },
        request_home(){
            let client = new ROSLIB.Service({
                ros: this.ros,
                name: this.btService,
                serviceType: 'rb1_autonomy_msg/TickBT'
            })
            console.log('sending server request to ' + client.name);
            
            let request = new ROSLIB.ServiceRequest({
                bt_xml_file : this.btXML_home
            });

            this.serviceIdleState = false;
            client.callService(request, (result) => {
                console.log('Result for service call on ' + client.name + ': ' + result.bt_status);
                this.serviceIdleState = true;
                this.serviceResponseFull = result.bt_status ? 'Succeeded' : 'Failure';
            }, (error) => {
                console.error(error);
                this.serviceIdleState = true;
            });
        },        
        get_topics(){
            let topicsClient = new ROSLIB.Service({
            ros : this.ros,
            name : '/rosapi/topics',
            serviceType : 'rosapi_msgs/Topics'
            });
            console.log('sending server request to ' + topicsClient.name);

            let request = new ROSLIB.ServiceRequest();
            topicsClient.callService(request, (result) => {
            console.log("Getting topics...");
            // get topic string array
            this.topics = result.topics;
            console.log(this.topics);
            });
        },
        get_services(){
            let topicsClient = new ROSLIB.Service({
            ros : this.ros,
            name : '/rosapi/services',
            serviceType : 'rosapi_msgs/Services'
            });
            console.log('sending server request to ' + topicsClient.name);

            let request = new ROSLIB.ServiceRequest();
            topicsClient.callService(request, (result) => {
            console.log("Getting topics...");
            // get service string array
            this.services = result.services;
            console.log(this.services);
            });
        
        },
        topicAvailable(topic) {
            const topicStatus = this.topics.find(item => item === topic);
            return topicStatus === topic;
        },
        serviceAvailable(service){
            const serviceStatus = this.services.find(item => item === service);
            return serviceStatus === service;
        },

    },
    mounted() {
    
        console.log('page is ready!');
        window.addEventListener('mouseup', this.stopDrag);
        document.getElementById('chkSwitch1').addEventListener('change', (e) => {
        if (e.target.checked) {
            console.log('Elevator Up');
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: this.liftUpTopic,
                messageType: 'std_msgs/String'
            });
            let message = new ROSLIB.Message({
            
                data: ""
            });
            topic.publish(message);
        } else {
            console.log('Elevator Down');
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: this.liftDownTopic,
                messageType: 'std_msgs/String'
            });
            let message = new ROSLIB.Message({
            
                data: ""
            });
            topic.publish(message);
        }
        });

    }

}).mount('#vueApp')
