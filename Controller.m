classdef Controller
    properties
        flight_path_angle_rad
        gravity_turn_init_altitude_km

    end
    
    methods
        function obj = Controller(inputArg1,inputArg2)
            %CONTROLLER Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function obj = velocity_controller(obj, ref_velocity_km_s     , ...
                                                velocity_vector_a_km_s)

        end

        function obj = fligth_path_angle_controller(obj )
        end
    end
end

