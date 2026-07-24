function urdf_to_simulink(urdf_filename, model_name)
    % URDF_TO_SIMULINK Imports a URDF file and builds a Simscape Multibody model.
    % Inputs:
    %   - urdf_filename: String path to the target .urdf file
    %   - model_name: String desired name for the output Simulink model
    
    if nargin < 1
        urdf_filename = 'universalUR5.urdf'; 
    end
    if nargin < 2
        model_name = 'simscape_robot_model'; 
    end

    % 1. Verify file existence
    if ~exist(urdf_filename, 'file')
        error('The specified URDF file "%s" was not found.', urdf_filename);
    end

    fprintf('Parsing "%s" and generating Simscape block structures...\n', urdf_filename);

    %% 2. Run the Simscape Multibody Importer
    % We temporarily disable the 'dataFileName' warning to keep the command window clean.
    warnState = warning('off', 'sm:import:UrdfDataFileNameNotSupported');
    
    try
        smimport(urdf_filename, 'ModelName', model_name);
    catch ME
        warning(warnState); % Restore warnings if it crashes
        error('Simscape Multibody import failed. Reason: %s', ME.message);
    end
    
    warning(warnState); % Restore normal warning behavior
    
    % Verify the model loaded successfully
    if bdIsLoaded(model_name)
        open_system(model_name);
    else
        error('Model was not successfully loaded into the Simulink architecture.');
    end

    %% 3. Apply Solver Settings
    fprintf('Configuring model solver settings for high-performance mechanics...\n');
    set_param(model_name, 'Solver', 'ode15s');
    set_param(model_name, 'MaxStep', '0.01');
    set_param(model_name, 'StartTime', '0.0');
    set_param(model_name, 'StopTime', '10.0'); 

    %% 4. Auto-Arrange Diagram Layout (FIXED FOR MODERN MATLAB)
    fprintf('Arranging block diagram architecture layout...\n');
    try
        % Modern API command to automatically straighten wires and clean up layout
        Simulink.BlockDiagram.arrangeSystem(model_name);
    catch
        % Safe fallback if working in an environment that restricts layout generation
        warning('Could not auto-arrange the diagram layout. The model is still completely valid.');
    end

    %% 5. Save the generated model
    save_system(model_name);
    fprintf('\n=== SUCCESS ===\n');
    fprintf('Simulink Model "%s.slx" has been created and saved successfully.\n', model_name);
end