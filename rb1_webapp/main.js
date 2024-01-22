const { createApp } = Vue

createApp({

    data() {
    
        return {
            ros_address: '', 
            connected: false, // status connection to ROS
            status: 'Connect', // text message shown
            menu_title: 'menu',
            main_title: 'main',
            paragraph1: 'paragraph1',
            paragraph2: 'paragraph2'
        };
    },
    methods: {
        /* get ros address from user */
        get_address(e){
            this.ros_address = e.target.value;
        },
        /* toggle connection button */
        toggle(){
            if (this.ros_address !== ''){
                this.connected = !this.connected;
                if (this.connected === false) {
                    this.status = 'Connect';
                }else{
                    this.status = 'Disconnect';
                }
            }
        }
    
    
    },

}).mount('#vueApp')
