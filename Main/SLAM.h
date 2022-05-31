class SLAM
{
    public:
        void Run()
        {
            StoreCurrentState();
            SystemState* state = &state_list[state_index % n]
            
            
            
        }

        void StoreCurrentState(float* x, float* y, float* yaw, 
            float* ir_dist1, float* ir_dist2, float* ir_dist3, float* ir_dist4,
            float* sonar_dist)
        {
            SystemState newState;
            newState.x = *x;
            newState.y = *y;
            newState.yaw = *yaw;
            float ir_dist[4] = {*ir_dist1, *ir_dist2, *ir_dist3, *ir_dist4};
            memcpy(newState.ir_dist, ir_dist, sizeof(newState.ir_dist));
            newState.sonar_dist = *sonar_dist;

            state_list[state_index % n] = newState;
            state_index++;
        }

        void PrintMap()
        {
            for (size_t i = 0; i < sizeof(state_list); i++)
            {
                SystemState* state = &state_list[i];
                float x_global = cos(state->yaw)*state->ir_dist[0] + state->x; //how to deal with ir offsets
                float y_global = cos(state->yaw)*state->ir_dist[0] + state->y;

                //IR1_point coords
                //IR2_point coords
                //IR3_point coords
                //IR4_point coords

                //sonar coords//
                //putting the sonar sensor on the turret might still be better to generate forward map of environment

                // can i find linear regions of a 360 map??

                Serial.println();
            }
            
        }

        void RunDataAssociate()
        {

        }

    private:

        float gradient = 0;

        const static int n = 1000; //number of previous states to save in map

        typedef struct{
            float x, y;
            int ID;
            int decayTime = 0; //time before landmark removed
        } Landmark;
        static short LandmarkID;

        typedef struct{
            float x, y, yaw;
            float ir_dist[4];
            float sonar_dist;
        } SystemState;

        SystemState state_list[n];
        int state_index = 0;

};