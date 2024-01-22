const { createApp } = Vue

createApp({

    data() {
    
        return {
            // ROS connection
            ros: null,
            rosbridge_address: 'wss://i-0f5a405e2fa8a7ef3.robotigniteacademy.com/720cdbb2-33c6-44bb-adb5-75b8fceb4efb/rosbridge/', 
            
            // button status
            connected: false,
            status: 'Connect',
            // content
            main_title1: 'Manual Control',
            main_title2: 'Autonomy',
            paragraph1: 'paragraph1',
            paragraph2: 'paragraph2',

            // velocity control
            selectedSpeed: 0.0,
            selectedTurn: 0.0,

            // select robot
            selectedEnvi: 'Simulated Robot',
            velTopic: '',
            liftUpTopic: '',
            liftDownTopic: '',
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
            });

            this.ros.on('close', () => {
            
                console.log('Connection closed!');
                this.connected = false;
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
        /* speed selector */
        onSelectSpeed(e){
            this.selectedSpeed = Number(e.target.value);
            console.log(`Current speed: ${this.selectedSpeed}`);
        },
        /* turn selector */
        onSelectTurn(e){
            this.selectedTurn = Number(e.target.value);
            console.log(`Current speed: ${this.selectedTurn}`);
        },
        onSelectEnvi(e){
            this.selectedEnvi = e.target.value;

            // set robot environment parameters here!
            if (this.selectedEnvi === 'Simulated Robot') {
                this.velTopic = '/diffbot_base_controller/cmd_vel_unstamped';
                this.liftUpTopic = '/elevator_up';
                this.liftDownTopic = '/elevator_down';
                
            } else if(this.selectedEnvi === 'Real Robot'){
                this.velTopic = '/cmd_vel';
                this.liftUpTopic = '/elevator_up';
                this.liftDownTopic = '/elevator_down';
            }
        },        
        /* send velocity command */
        goForward(){
        
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: this.velTopic,
                messageType: 'geometry_msgs/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: this.selectedSpeed, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0, },
            })
            topic.publish(message)
        },
        goBackward(){
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: this.velTopic,
                messageType: 'geometry_msgs/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: (-1)*this.selectedSpeed, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0, },
            })
            topic.publish(message)
        
        },
        turnLeft(){
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: this.velTopic,
                messageType: 'geometry_msgs/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: 0, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: this.selectedTurn, },
            })
            topic.publish(message)

        },
        turnRight(){
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: this.velTopic,
                messageType: 'geometry_msgs/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: 0, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: (-1)*this.selectedTurn, },
            })
            topic.publish(message)

        },
        /* send elevator command */
        activateLift(){
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: this.liftUpTopic,
                messageType: 'std_msgs/String'
            });
            let message = new ROSLIB.Message({
            
                data: ""
            });
            topic.publish(message);
        },
        deactivateLift(){
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
    
    },
    mounted() {
    
        console.log('page is ready!');

    }

}).mount('#vueApp')
