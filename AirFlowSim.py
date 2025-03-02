import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from PyQt5 import QtWidgets, QtCore

# Check for optional VTK-based visualization
try:
    import vtk
    from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
    VTK_AVAILABLE = True
except ImportError:
    VTK_AVAILABLE = False
    print("VTK not available. Will use matplotlib for visualization.")

class AirFlowSimApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AirFlowSim - CAD Aerodynamic Simulator")
        self.resize(1200, 800)
        
        # Initialize variables
        self.model_path = None
        self.model_mesh = None
        self.simulation_results = None
        self.wind_speed = 10.0  # m/s
        self.temperature = 293.15  # K (20°C)
        self.pressure = 101325  # Pa (1 atm)
        self.visualization_type = "particles"  # or "streamlines"
        
        # Create the UI
        self.setup_ui()
        
    def setup_ui(self):
        # Create central widget and layout
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout(central_widget)
        
        # Create the 3D view (either VTK or Matplotlib)
        if VTK_AVAILABLE:
            self.vtk_widget = QVTKRenderWindowInteractor(central_widget)
            main_layout.addWidget(self.vtk_widget, 3)
            
            # Set up the renderer
            self.renderer = vtk.vtkRenderer()
            self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)
            self.renderer.SetBackground(0.2, 0.2, 0.2)
            
            # Initialize interaction
            self.interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
            self.interactor.Initialize()
        else:
            # Fallback to matplotlib
            self.figure = plt.figure(figsize=(8, 6))
            self.canvas = FigureCanvas(self.figure)
            self.ax = self.figure.add_subplot(111, projection='3d')
            main_layout.addWidget(self.canvas, 3)
        
        # Create control panel
        control_panel = QtWidgets.QWidget()
        control_layout = QtWidgets.QVBoxLayout(control_panel)
        main_layout.addWidget(control_panel, 1)
        
        # File import section
        file_group = QtWidgets.QGroupBox("Import Model")
        file_layout = QtWidgets.QVBoxLayout(file_group)
        
        import_button = QtWidgets.QPushButton("Import CAD Model")
        import_button.clicked.connect(self.import_model)
        file_layout.addWidget(import_button)
        
        self.file_label = QtWidgets.QLabel("No file loaded")
        file_layout.addWidget(self.file_label)
        
        control_layout.addWidget(file_group)
        
        # Simulation parameters
        sim_group = QtWidgets.QGroupBox("Simulation Parameters")
        sim_layout = QtWidgets.QVBoxLayout(sim_group)
        
        # Wind speed
        wind_layout = QtWidgets.QHBoxLayout()
        wind_layout.addWidget(QtWidgets.QLabel("Wind Speed (m/s):"))
        self.wind_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.wind_slider.setRange(0, 100)
        self.wind_slider.setValue(int(self.wind_speed))
        self.wind_slider.valueChanged.connect(self.update_wind_speed)
        wind_layout.addWidget(self.wind_slider)
        self.wind_label = QtWidgets.QLabel(f"{self.wind_speed:.1f}")
        wind_layout.addWidget(self.wind_label)
        sim_layout.addLayout(wind_layout)
        
        # Temperature
        temp_layout = QtWidgets.QHBoxLayout()
        temp_layout.addWidget(QtWidgets.QLabel("Temperature (°C):"))
        self.temp_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.temp_slider.setRange(-50, 50)
        self.temp_slider.setValue(int(self.temperature - 273.15))
        self.temp_slider.valueChanged.connect(self.update_temperature)
        temp_layout.addWidget(self.temp_slider)
        self.temp_label = QtWidgets.QLabel(f"{self.temperature - 273.15:.1f}")
        temp_layout.addWidget(self.temp_label)
        sim_layout.addLayout(temp_layout)
        
        # Visualization type
        viz_layout = QtWidgets.QHBoxLayout()
        viz_layout.addWidget(QtWidgets.QLabel("Visualization:"))
        self.viz_combo = QtWidgets.QComboBox()
        self.viz_combo.addItems(["Particles", "Streamlines"])
        self.viz_combo.currentTextChanged.connect(self.update_visualization_type)
        viz_layout.addWidget(self.viz_combo)
        sim_layout.addLayout(viz_layout)
        
        control_layout.addWidget(sim_group)
        
        # Run simulation button
        self.run_button = QtWidgets.QPushButton("Run Simulation")
        self.run_button.clicked.connect(self.run_simulation)
        self.run_button.setEnabled(False)
        control_layout.addWidget(self.run_button)
        
        # Export results button
        self.export_button = QtWidgets.QPushButton("Export Results")
        self.export_button.clicked.connect(self.export_results)
        self.export_button.setEnabled(False)
        control_layout.addWidget(self.export_button)
        
        # Status bar
        self.statusBar().showMessage("Ready")
        
        # Initialize 3D view with text
        if VTK_AVAILABLE:
            text_actor = vtk.vtkTextActor()
            text_actor.SetInput("Import a CAD model to begin simulation")
            text_actor.GetTextProperty().SetFontSize(14)
            text_actor.GetTextProperty().SetColor(1.0, 1.0, 1.0)
            text_actor.SetPosition(10, 10)
            self.renderer.AddActor(text_actor)
            self.text_actor = text_actor
        else:
            self.ax.text(0, 0, 0, "Import a CAD model to begin simulation", 
                        fontsize=14, ha='center')
            self.canvas.draw()
    
    def import_model(self):
        file_dialog = QtWidgets.QFileDialog()
        file_path, _ = file_dialog.getOpenFileName(
            self, "Import CAD Model", "", 
            "CAD Files (*.stl *.obj);;All Files (*)"
        )
        
        if file_path:
            self.statusBar().showMessage(f"Loading model: {file_path}")
            try:
                self.model_path = file_path
                self.file_label.setText(os.path.basename(file_path))
                
                # Load the model based on file extension
                file_ext = os.path.splitext(file_path)[1].lower()
                
                if VTK_AVAILABLE:
                    # Use VTK to load the model
                    if file_ext == '.stl':
                        reader = vtk.vtkSTLReader()
                        reader.SetFileName(file_path)
                        reader.Update()
                        self.model_mesh = reader.GetOutput()
                    elif file_ext == '.obj':
                        reader = vtk.vtkOBJReader()
                        reader.SetFileName(file_path)
                        reader.Update()
                        self.model_mesh = reader.GetOutput()
                    else:
                        raise ValueError(f"Unsupported file format: {file_ext}")
                    
                    # Create a mapper and actor
                    mapper = vtk.vtkPolyDataMapper()
                    mapper.SetInputData(self.model_mesh)
                    
                    self.model_actor = vtk.vtkActor()
                    self.model_actor.SetMapper(mapper)
                    self.model_actor.GetProperty().SetColor(0.8, 0.8, 0.8)
                    
                    # Clear the renderer and add the model
                    self.renderer.RemoveAllViewProps()
                    self.renderer.AddActor(self.model_actor)
                    
                    # Reset camera
                    self.renderer.ResetCamera()
                    self.vtk_widget.GetRenderWindow().Render()
                    
                else:
                    # Simple placeholder for non-VTK mode
                    # In a real implementation, you'd use another library to load the mesh
                    self.model_mesh = {"path": file_path}
                    
                    # Clear the plot and show a simple placeholder
                    self.ax.clear()
                    self.ax.text(0, 0, 0, f"Model loaded: {os.path.basename(file_path)}", 
                                fontsize=12, ha='center')
                    self.canvas.draw()
                
                # Enable simulation
                self.run_button.setEnabled(True)
                self.statusBar().showMessage(f"Model loaded: {os.path.basename(file_path)}")
                
            except Exception as e:
                self.statusBar().showMessage(f"Error loading model: {str(e)}")
                QtWidgets.QMessageBox.critical(self, "Import Error", f"Failed to load model: {str(e)}")
    
    def update_wind_speed(self, value):
        self.wind_speed = float(value)
        self.wind_label.setText(f"{self.wind_speed:.1f}")
        if self.simulation_results is not None:
            self.visualize_results()
    
    def update_temperature(self, value):
        self.temperature = float(value) + 273.15  # Convert to K
        self.temp_label.setText(f"{value:.1f}")
        if self.simulation_results is not None:
            self.visualize_results()
    
    def update_visualization_type(self, text):
        self.visualization_type = text.lower()
        if self.simulation_results is not None:
            self.visualize_results()
    
    def run_simulation(self):
        if self.model_mesh is None:
            return
        
        self.statusBar().showMessage("Running simulation...")
        self.run_button.setEnabled(False)
        
        try:
            # Simple simulation logic
            if VTK_AVAILABLE:
                # Get model bounds
                bounds = [0] * 6
                self.model_mesh.GetBounds(bounds)
                
                # Create a structured grid for simulation
                resolution = 20
                
                # Domain extends in all directions
                domain_min = [
                    bounds[0] - (bounds[1] - bounds[0]), 
                    bounds[2] - (bounds[3] - bounds[2]), 
                    bounds[4] - (bounds[5] - bounds[4])
                ]
                domain_max = [
                    bounds[1] + 2 * (bounds[1] - bounds[0]), 
                    bounds[3] + (bounds[3] - bounds[2]), 
                    bounds[5] + (bounds[5] - bounds[4])
                ]
                
                # Create a structured grid
                grid = vtk.vtkStructuredGrid()
                grid_points = vtk.vtkPoints()
                
                # Create grid geometry
                dimensions = [resolution, resolution, resolution]
                grid.SetDimensions(dimensions)
                
                # Create points
                for k in range(dimensions[2]):
                    z = domain_min[2] + k * (domain_max[2] - domain_min[2]) / (dimensions[2] - 1)
                    for j in range(dimensions[1]):
                        y = domain_min[1] + j * (domain_max[1] - domain_min[1]) / (dimensions[1] - 1)
                        for i in range(dimensions[0]):
                            x = domain_min[0] + i * (domain_max[0] - domain_min[0]) / (dimensions[0] - 1)
                            grid_points.InsertNextPoint(x, y, z)
                
                grid.SetPoints(grid_points)
                
                # Create velocity field
                velocity = vtk.vtkFloatArray()
                velocity.SetNumberOfComponents(3)
                velocity.SetName("velocity")
                
                # Create pressure field
                pressure = vtk.vtkFloatArray()
                pressure.SetNumberOfComponents(1)
                pressure.SetName("pressure")
                
                # Simple flow field computation (uniform flow in x direction with perturbation)
                locator = vtk.vtkCellLocator()
                locator.SetDataSet(self.model_mesh)
                locator.BuildLocator()
                
                for i in range(grid_points.GetNumberOfPoints()):
                    point = [0, 0, 0]
                    grid_points.GetPoint(i, point)
                    
                    # Initialize with uniform flow
                    vx = self.wind_speed
                    vy = 0.0
                    vz = 0.0
                    
                    # Check if point is close to the model
                    closest_point = [0, 0, 0]
                    cell_id = vtk.reference(0)  # Use reference instead of mutable
                    sub_id = vtk.reference(0)  # Use reference instead of mutable
                    dist2 = vtk.reference(0.0)  # Use reference instead of mutable
                    
                    locator.FindClosestPoint(point, closest_point, cell_id, sub_id, dist2)
                    
                    # Manual conversion from VTK reference to float
                    distance = float(np.sqrt(float(dist2)))  # First convert to float then use numpy
                    
                    # Perturb velocity field based on distance to model
                    if distance < 1.0:
                        # Slow down flow near the model
                        factor = distance
                        vx *= factor
                        
                        # Calculate deflection direction (simple approximation)
                        direction = [
                            point[0] - closest_point[0],
                            point[1] - closest_point[1],
                            point[2] - closest_point[2]
                        ]
                        
                        # Normalize direction
                        length = np.sqrt(direction[0]**2 + direction[1]**2 + direction[2]**2)
                        if length > 0:
                            direction = [d/length for d in direction]
                            
                            # Add deflection to velocity
                            deflection_strength = (1.0 - factor) * self.wind_speed * 0.5
                            vy += direction[1] * deflection_strength
                            vz += direction[2] * deflection_strength
                    
                    # Calculate pressure using simplified Bernoulli
                    v_squared = vx**2 + vy**2 + vz**2
                    v_inf_squared = self.wind_speed**2
                    density = 1.225  # Air density at sea level (kg/m³)
                    p = 0.5 * density * (v_inf_squared - v_squared)
                    
                    velocity.InsertNextTuple3(vx, vy, vz)
                    pressure.InsertNextTuple1(p)
                
                grid.GetPointData().AddArray(velocity)
                grid.GetPointData().AddArray(pressure)
                grid.GetPointData().SetActiveVectors("velocity")
                grid.GetPointData().SetActiveScalars("pressure")
                
                self.simulation_results = grid
                
                # Visualize results
                self.visualize_results()
                
            else:
                # Simple placeholder for non-VTK mode
                self.simulation_results = {"wind_speed": self.wind_speed, "temp": self.temperature}
                
                # Clear the plot and show a simple placeholder
                self.ax.clear()
                self.ax.text(0, 0, 0, "Simulation completed!", fontsize=14, ha='center')
                self.canvas.draw()
            
            self.export_button.setEnabled(True)
            self.statusBar().showMessage("Simulation completed")
            
        except Exception as e:
            self.statusBar().showMessage(f"Simulation error: {str(e)}")
            QtWidgets.QMessageBox.critical(self, "Simulation Error", f"Failed to run simulation: {str(e)}")
        finally:
            self.run_button.setEnabled(True)
    
    def visualize_results(self):
        if self.simulation_results is None:
            return
        
        if VTK_AVAILABLE:
            # Clear previous visualization actors
            actors_to_keep = [self.model_actor]
            for actor in self.renderer.GetActors():
                if actor not in actors_to_keep:
                    self.renderer.RemoveActor(actor)
            
            # Apply pressure to model surface
            pressure_filter = vtk.vtkProbeFilter()
            pressure_filter.SetInputData(self.model_mesh)
            pressure_filter.SetSourceData(self.simulation_results)
            pressure_filter.Update()
            
            # Create a color map for pressure
            lut = vtk.vtkLookupTable()
            lut.SetNumberOfTableValues(256)
            lut.SetHueRange(0.667, 0.0)  # Blue to red
            lut.SetTableRange(0, 100)  # Adjust based on your pressure range
            lut.Build()
            
            # Apply coloring to model
            model_mapper = self.model_actor.GetMapper()
            model_mapper.SetInputData(pressure_filter.GetOutput())
            model_mapper.SetScalarModeToUsePointFieldData()
            model_mapper.SelectColorArray("pressure")
            model_mapper.SetLookupTable(lut)
            model_mapper.SetScalarVisibility(True)
            
            # Add scalar bar (color legend)
            scalar_bar = vtk.vtkScalarBarActor()
            scalar_bar.SetLookupTable(lut)
            scalar_bar.SetTitle("Pressure (Pa)")
            scalar_bar.SetNumberOfLabels(5)
            scalar_bar.SetPosition(0.05, 0.05)
            scalar_bar.SetWidth(0.15)
            scalar_bar.SetHeight(0.9)
            scalar_bar.GetTitleTextProperty().SetColor(1, 1, 1)
            scalar_bar.GetLabelTextProperty().SetColor(1, 1, 1)
            self.renderer.AddActor(scalar_bar)
            
            # Visualize flow
            if self.visualization_type == "particles":
                # Create particle source points at inlet
                bounds = [0] * 6
                self.simulation_results.GetBounds(bounds)
                
                # Create points at inflow boundary
                particle_source = vtk.vtkPointSource()
                particle_source.SetCenter(bounds[0] + 0.1, (bounds[2] + bounds[3])/2, (bounds[4] + bounds[5])/2)
                particle_source.SetRadius(min(bounds[3]-bounds[2], bounds[5]-bounds[4]) * 0.4)
                particle_source.SetNumberOfPoints(100)
                particle_source.Update()
                
                # Create streamlines
                streamline = vtk.vtkStreamTracer()
                streamline.SetInputData(self.simulation_results)
                streamline.SetSourceData(particle_source.GetOutput())
                streamline.SetMaximumPropagation(100)
                streamline.SetIntegrationDirectionToForward()
                streamline.SetIntegratorTypeToRungeKutta45()
                streamline.Update()
                
                # Create spheres for particles
                glyph = vtk.vtkGlyph3D()
                sphere = vtk.vtkSphereSource()
                sphere.SetRadius(0.05)
                sphere.SetPhiResolution(8)
                sphere.SetThetaResolution(8)
                sphere.Update()
                
                glyph.SetInputData(streamline.GetOutput())
                glyph.SetSourceData(sphere.GetOutput())
                glyph.ScalingOff()
                glyph.Update()
                
                # Create mapper and actor for particles
                particle_mapper = vtk.vtkPolyDataMapper()
                particle_mapper.SetInputData(glyph.GetOutput())
                
                particle_actor = vtk.vtkActor()
                particle_actor.SetMapper(particle_mapper)
                particle_actor.GetProperty().SetColor(0.1, 0.9, 0.1)  # Green particles
                
                self.renderer.AddActor(particle_actor)
                
            else:  # streamlines
                # Create streamlines
                streamlines = vtk.vtkStreamTracer()
                
                # Create a plane of seed points
                bounds = [0] * 6
                self.simulation_results.GetBounds(bounds)
                
                # Create points at inflow boundary
                plane = vtk.vtkPlaneSource()
                plane.SetOrigin(bounds[0] + 0.1, bounds[2], bounds[4])
                plane.SetPoint1(bounds[0] + 0.1, bounds[3], bounds[4])
                plane.SetPoint2(bounds[0] + 0.1, bounds[2], bounds[5])
                plane.SetResolution(8, 8)
                plane.Update()
                
                streamlines.SetInputData(self.simulation_results)
                streamlines.SetSourceData(plane.GetOutput())
                streamlines.SetMaximumPropagation(200)
                streamlines.SetIntegrationDirectionToForward()
                streamlines.SetIntegratorTypeToRungeKutta45()
                streamlines.Update()
                
                # Create tubes for streamlines
                tubes = vtk.vtkTubeFilter()
                tubes.SetInputData(streamlines.GetOutput())
                tubes.SetRadius(0.02)
                tubes.SetNumberOfSides(12)
                tubes.SetVaryRadiusToVaryRadiusOff()
                tubes.Update()
                
                # Create mapper and actor for streamlines
                streamline_mapper = vtk.vtkPolyDataMapper()
                streamline_mapper.SetInputData(tubes.GetOutput())
                
                # Color streamlines by velocity magnitude
                streamline_mapper.ScalarVisibilityOn()
                streamline_mapper.SetScalarModeToUsePointFieldData()
                
                streamline_actor = vtk.vtkActor()
                streamline_actor.SetMapper(streamline_mapper)
                
                self.renderer.AddActor(streamline_actor)
            
            # Render the scene
            self.vtk_widget.GetRenderWindow().Render()
            
        else:
            # Simple placeholder for non-VTK mode
            self.ax.clear()
            self.ax.text(0, 0, 0, f"Visualization: {self.visualization_type}\nWind speed: {self.wind_speed} m/s", 
                        fontsize=12, ha='center')
            self.canvas.draw()
    
    def export_results(self):
        if self.simulation_results is None:
            return
            
        file_dialog = QtWidgets.QFileDialog()
        file_path, _ = file_dialog.getSaveFileName(
            self, "Export Results", "", 
            "VTK Files (*.vtk);;VTK Polydata (*.vtp);;All Files (*)"
        )
        
        if file_path:
            try:
                if VTK_AVAILABLE:
                    # Save the simulation results
                    writer = vtk.vtkStructuredGridWriter()
                    writer.SetInputData(self.simulation_results)
                    writer.SetFileName(file_path)
                    writer.Write()
                    
                    # Save the model with pressure data
                    model_path = os.path.splitext(file_path)[0] + "_model.vtk"
                    poly_writer = vtk.vtkPolyDataWriter()
                    poly_writer.SetInputData(self.model_actor.GetMapper().GetInput())
                    poly_writer.SetFileName(model_path)
                    poly_writer.Write()
                else:
                    # Simple placeholder for non-VTK mode
                    with open(file_path, 'w') as f:
                        f.write(f"Simulation results:\nWind speed: {self.wind_speed} m/s\nTemperature: {self.temperature} K\n")
                
                self.statusBar().showMessage(f"Results exported to {file_path}")
                
            except Exception as e:
                self.statusBar().showMessage(f"Export error: {str(e)}")
                QtWidgets.QMessageBox.critical(self, "Export Error", f"Failed to export results: {str(e)}")

# Add matplotlib support for non-VTK mode
if not VTK_AVAILABLE:
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from mpl_toolkits.mplot3d import Axes3D

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = AirFlowSimApp()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()