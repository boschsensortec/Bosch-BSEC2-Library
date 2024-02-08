The generic_examples are provided only for a single sensor implementation.
If multiple sensors are used kindly refer to x8_board_examples where support for 8 sensors are demonstrated.


basic_config_state.ino example supports regression as well as classification outputs.
However BSEC supports only one mode at any given point of time.
Ensure to define the output mode required in the example code before compilation.

    Set the OUTPUT_MODE macro to CLASSIFICATION for the classification output of the BSEC algorithm (default).
        #define OUTPUT_MODE		CLASSIFICATION
    Set the OUTPUT_MODE macro to REGRESSION for the regression output of the BSEC algorithm.
        #define OUTPUT_MODE		REGRESSION