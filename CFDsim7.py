import sys
import os
import numpy as np
import vtk
import trimesh
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import time

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QPushButton, QSlider, QLabel, QFileDialog, 
                            QComboBox, QGroupBox, QFormLayout, QDoubleSpinBox,
                            QCheckBox, QProgressBar, QMessageBox)
from PyQt5.QtCore import Qt, QTimer

# Try different import methods for VTK's Qt integration
vtk_qt_available = False
try:
    from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
    vtk_qt_available = True
except ImportError:
    try:
        from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
        vtk_qt_available = True
    except ImportError:
        print("Could not import VTK Qt integration. Some features may be limited.")


class SimplifiedCFDSolver:
    """
    A simplified CFD solver for demonstration purposes.
    In a real application, this would interface with more sophisticated 
    CFD libraries like FEniCS, OpenFOAM, etc.
    """
    def __init__(self):
        self.mesh = None
        self.airspeed = 10.0  # m/s default
        self.angle = 0.0      # angle of attack in degrees (pitch)
        self.yaw = 0.0        # yaw angle in degrees (horizontal rotation)
        self.density = 1.225  # air density kg/m^3
        self.viscosity = 1.81e-5  # air viscosity kg/(m*s)
        self.reference_area = 1.0  # m^2
        self.reference_length = 1.0  # m
        
    def set_mesh(self, mesh):
        self.mesh = mesh
        # Calculate reference values from the mesh
        self.reference_area = mesh.area
        self.reference_length = max(mesh.extents)
        
    def set_airspeed(self, speed):
        self.airspeed = speed
        
    def set_angle(self, angle):
        self.angle = angle
    
    def set_yaw(self, yaw):
        self.yaw = yaw
    
    def set_density(self, density):
        self.density = density
        
    def calculate_reynolds_number(self):
        """Calculate Reynolds number based on reference length and flow properties"""
        return (self.density * self.airspeed * self.reference_length) / self.viscosity
        
    def calculate_pressure(self):
        """
        CFD calculation with improved realism and error handling.
        
        This improved implementation focuses on creating realistic pressure distributions
        while protecting against numerical issues.
        """
        if self.mesh is None:
            return None
            
        try:
            # Get mesh face normals
            face_normals = self.mesh.face_normals
            
            # Create a flow direction vector based on angle of attack and yaw
            pitch_rad = np.radians(self.angle)
            yaw_rad = np.radians(self.yaw)
            
            # Calculate direction components
            x = np.cos(pitch_rad) * np.cos(yaw_rad)
            y = np.cos(pitch_rad) * np.sin(yaw_rad)
            z = np.sin(pitch_rad)
            
            flow_dir = np.array([x, y, z])
            
            # Calculate dot product between face normals and flow direction
            # Negative dot product means the face is facing into the flow
            dots = np.dot(face_normals, flow_dir)
            
            # Handle the case of zero airspeed
            if self.airspeed <= 0.01:  # Minimum threshold to avoid division by zero
                return np.zeros(len(dots))
                
            # Dynamic pressure (q = 0.5 * rho * V^2)
            dyn_pressure = 0.5 * self.density * self.airspeed**2
            
            # Initialize pressure coefficients
            cp = np.zeros(len(dots))
            
            # Stagnation points (faces directly against the flow)
            # These should have high positive pressure (red)
            stagnation_mask = dots < -0.8
            cp[stagnation_mask] = 1.0 + 0.6 * dots[stagnation_mask]  # Amplify stagnation pressure
            
            # Separation/wake regions (faces roughly perpendicular to flow)
            # These should have moderate negative pressure (green/blue)
            separation_mask = (dots >= -0.8) & (dots < -0.1)
            cp[separation_mask] = 0.9 * dots[separation_mask] - 0.1
            
            # Low pressure/suction regions (faces parallel or away from flow)
            # These should have more extreme negative pressure (deep blue)
            suction_mask = (dots >= -0.1) & (dots < 0.3)
            cp[suction_mask] = 2.5 * dots[suction_mask] - 0.3
            
            # Wake regions (faces fully away from flow)
            wake_mask = dots >= 0.3
            cp[wake_mask] = 0.5 * dots[wake_mask] - 0.5
            
            # Add curvature effects for more realistic flow separation
            # Get face centers to find position
            face_centers = self.mesh.triangles_center
            
            # Forward-facing areas should have high pressure
            x_position = face_centers[:, 0]  # Front-to-back position
            z_position = face_centers[:, 2]  # Height position
            
            # Calculate mean and std safely (avoid NaN or inf)
            x_mean = np.mean(x_position) if len(x_position) > 0 else 0
            x_std = np.std(x_position) if len(x_position) > 0 and np.std(x_position) > 0 else 1.0
            z_mean = np.mean(z_position) if len(z_position) > 0 else 0
            z_std = np.std(z_position) if len(z_position) > 0 and np.std(z_position) > 0 else 1.0
            
            # Adjust front faces (especially the nose/bumper) to have higher pressure
            front_mask = (x_position < x_mean - 0.4 * x_std) & (cp > -0.5)
            cp[front_mask] = np.maximum(cp[front_mask], -0.3) * 1.5
            
            # Create low pressure zone on top surfaces (roof)
            roof_mask = (z_position > z_mean + 0.3 * z_std) & (dots > -0.7)
            cp[roof_mask] = np.minimum(cp[roof_mask], 0.0) * 1.3 - 0.3
            
            # Create low pressure at rear (wake)
            rear_mask = x_position > x_mean + 0.4 * x_std
            cp[rear_mask] = np.minimum(cp[rear_mask], 0.0) * 1.2 - 0.2
            
            # Calculate actual pressures from pressure coefficients and dynamic pressure
            pressures = cp * dyn_pressure
            
            # Check for NaN or infinite values and replace them
            pressures = np.nan_to_num(pressures, nan=0.0, posinf=dyn_pressure, neginf=-dyn_pressure)
            
            return pressures
            
        except Exception as e:
            print(f"Error in pressure calculation: {str(e)}")
            import traceback
            traceback.print_exc()
            # Return zero array as fallback
            return np.zeros(len(self.mesh.face_normals))
    
    def generate_streamlines(self, domain_size=10, grid_density=10):
        """
        Generate simplified streamlines based on the model shape.
        
        In a real CFD application, streamlines would come from the velocity field
        solution of the Navier-Stokes equations.
        """
        if self.mesh is None:
            return None, None
            
        # Create a grid of starting points for streamlines
        x_start = -domain_size/2
        y_range = np.linspace(-domain_size/2, domain_size/2, grid_density)
        z_range = np.linspace(0.1, domain_size/2, grid_density)
        
        streamlines = []
        streamline_colors = []
        
        # Flow direction based on angle of attack and yaw
        pitch_rad = np.radians(self.angle)
        yaw_rad = np.radians(self.yaw)
        
        # Calculate direction components
        x = np.cos(pitch_rad) * np.cos(yaw_rad)
        y = np.cos(pitch_rad) * np.sin(yaw_rad)
        z = np.sin(pitch_rad)
        
        flow_dir = np.array([x, y, z])
        
        # For each starting point, generate a streamline
        for y in y_range:
            for z in z_range:
                # Start point
                point = np.array([x_start, y, z])
                streamline = [point]
                
                # Simple streamline integration (this is highly simplified)
                for i in range(100):  # Maximum 100 steps per streamline
                    # Calculate the next point by adding the flow direction
                    # and a small disturbance based on distance to the model
                    
                    # Current point
                    curr_point = streamline[-1]
                    
                    # Find minimum distance to the mesh (simplified)
                    distances = np.linalg.norm(self.mesh.vertices - curr_point, axis=1)
                    min_dist = np.min(distances)
                    
                    # Disturb the flow direction based on distance to the mesh
                    # (this creates a simple flow deflection around the object)
                    disturbance = np.zeros(3)
                    if min_dist < domain_size/4:
                        # Find the closest vertex
                        closest_idx = np.argmin(distances)
                        closest_vertex = self.mesh.vertices[closest_idx]
                        
                        # Direction from point to vertex
                        to_vertex = closest_vertex - curr_point
                        to_vertex_norm = np.linalg.norm(to_vertex)
                        if to_vertex_norm > 0:
                            to_vertex = to_vertex / to_vertex_norm
                            
                        # Create a deflection perpendicular to both flow and to_vertex
                        deflection = np.cross(flow_dir, to_vertex)
                        if np.linalg.norm(deflection) > 0:
                            deflection = deflection / np.linalg.norm(deflection)
                            
                        # Scale the deflection by distance (closer = stronger)
                        strength = 1.0 - min(min_dist / (domain_size/4), 1.0)
                        disturbance = deflection * strength * 0.5
                    
                    # Calculate new position
                    step_size = 0.1 * (1.0 + min_dist / domain_size)  # Variable step size
                    next_point = curr_point + (flow_dir + disturbance) * step_size
                    
                    # Check if we're past the domain
                    if (next_point[0] > domain_size/2 or 
                        np.abs(next_point[1]) > domain_size/2 or 
                        next_point[2] > domain_size/2 or
                        next_point[2] < 0):
                        break
                    
                    # Check for collision with the mesh (very simplified)
                    if min_dist < 0.05:  # If very close to the mesh, stop
                        break
                        
                    # Add the point to the streamline
                    streamline.append(next_point)
                
                # Calculate a color based on the streamline's starting position
                # This gives a nice rainbow effect to differentiate streamlines
                r = abs(y) / (domain_size/2)
                g = abs(z) / (domain_size/2)
                b = 1.0 - 0.5 * (r + g)
                streamline_colors.append([r, g, b])
                
                # Convert to numpy array
                streamline = np.array(streamline)
                streamlines.append(streamline)
        
        return streamlines, streamline_colors


class CFDVisualizationApp(QMainWindow):
    """
    Main application window for the CFD visualization tool.
    """
    def __init__(self):
        super().__init__()
        self.cfd_solver = SimplifiedCFDSolver()
        self.model_actor = None
        self.streamlines_actor = None
        self.pressure_mapped = False
        self.current_model_path = None
        self.analysis_running = False
        self.streamlines_visible = False
        
        # Initialize the UI
        self.initUI()
        
    def initUI(self):
        """Set up the user interface"""
        self.setWindowTitle("Python CFD Visualization App")
        self.setGeometry(100, 100, 1200, 800)
        
        # Main layout with control panel and visualization area
        main_widget = QWidget()
        main_layout = QHBoxLayout()
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)
        
        # Create VTK widget for 3D visualization
        if vtk_qt_available:
            vtk_widget = QVTKRenderWindowInteractor()
            self.vtk_widget = vtk_widget
            
            # Set up VTK renderer
            self.renderer = vtk.vtkRenderer()
            self.renderer.SetBackground(0.2, 0.3, 0.4)
            vtk_widget.GetRenderWindow().AddRenderer(self.renderer)
            self.iren = vtk_widget.GetRenderWindow().GetInteractor()
            
            # Add a ground plane
            self.add_ground_plane()
            
            # Set up camera
            self.camera = self.renderer.GetActiveCamera()
            self.camera.SetPosition(0, -10, 3)
            self.camera.SetFocalPoint(0, 0, 0)
            self.camera.SetViewUp(0, 0, 1)
            
            # Add axes for reference
            self.axes = vtk.vtkAxesActor()
            self.axes.SetTotalLength(1.0, 1.0, 1.0)
            self.axes.SetShaftType(0)
            self.axes.SetAxisLabels(1)
            self.axes.SetCylinderRadius(0.02)
            self.renderer.AddActor(self.axes)
        else:
            # Create a placeholder widget if VTK Qt integration is not available
            vtk_widget = QWidget()
            self.vtk_widget = vtk_widget
            QLabel("VTK Qt integration not available", vtk_widget).setAlignment(Qt.AlignCenter)
            vtk_widget.setStyleSheet("background-color: #345; color: white;")
            self.renderer = None
        
        # Control panel
        control_panel = QWidget()
        control_layout = QVBoxLayout()
        control_panel.setLayout(control_layout)
        control_panel.setFixedWidth(350)
        
        # ---- File Operations Group ----
        file_group = QGroupBox("File Operations")
        file_layout = QVBoxLayout()
        
        # Import model button
        import_btn = QPushButton("Import Model (.stl/.obj)")
        import_btn.clicked.connect(self.import_model)
        file_layout.addWidget(import_btn)
        
        # Screenshot button
        screenshot_btn = QPushButton("Take Screenshot")
        screenshot_btn.clicked.connect(self.take_screenshot)
        file_layout.addWidget(screenshot_btn)
        
        file_group.setLayout(file_layout)
        control_layout.addWidget(file_group)
        
        # ---- Flow Parameters Group ----
        flow_group = QGroupBox("Flow Parameters")
        flow_layout = QFormLayout()
        
        # Airspeed input
        self.airspeed_spin = QDoubleSpinBox()
        self.airspeed_spin.setRange(0.0, 350.0)  # Allow 0 to 350 m/s
        self.airspeed_spin.setValue(10.0)
        self.airspeed_spin.setSuffix(" m/s")
        self.airspeed_spin.setDecimals(2)  # Allow more precision
        self.airspeed_spin.setSingleStep(5.0)  # Step by 5 m/s for easier adjustment
        self.airspeed_spin.valueChanged.connect(self.update_airspeed)
        flow_layout.addRow("Airspeed:", self.airspeed_spin)
        
        # Angle of attack input (pitch)
        self.angle_spin = QDoubleSpinBox()
        self.angle_spin.setRange(-90.0, 90.0)
        self.angle_spin.setValue(0.0)
        self.angle_spin.setSuffix("°")
        self.angle_spin.valueChanged.connect(self.update_angle)
        flow_layout.addRow("Angle of Attack (Pitch):", self.angle_spin)
        
        # Yaw angle input (horizontal rotation) - Slider version
        yaw_layout = QHBoxLayout()
        
        self.yaw_slider = QSlider(Qt.Horizontal)
        self.yaw_slider.setRange(-180, 180)
        self.yaw_slider.setValue(0)
        self.yaw_slider.setTickPosition(QSlider.TicksBelow)
        self.yaw_slider.setTickInterval(30)
        self.yaw_slider.valueChanged.connect(self.update_yaw_from_slider)
        
        self.yaw_label = QLabel("0°")
        self.yaw_label.setMinimumWidth(40)
        self.yaw_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        
        yaw_layout.addWidget(self.yaw_slider)
        yaw_layout.addWidget(self.yaw_label)
        
        flow_layout.addRow("Flow Direction (Yaw):", yaw_layout)
        
        # Air density input
        self.density_spin = QDoubleSpinBox()
        self.density_spin.setRange(0.1, 10.0)
        self.density_spin.setValue(1.225)
        self.density_spin.setSuffix(" kg/m³")
        self.density_spin.valueChanged.connect(self.update_density)
        flow_layout.addRow("Air Density:", self.density_spin)
        
        flow_group.setLayout(flow_layout)
        control_layout.addWidget(flow_group)
        
        # ---- Analysis Controls Group ----
        analysis_group = QGroupBox("Analysis Controls")
        analysis_layout = QVBoxLayout()
        
        # Run analysis button
        self.analyze_btn = QPushButton("Run CFD Analysis")
        self.analyze_btn.clicked.connect(self.run_cfd_analysis)
        self.analyze_btn.setEnabled(False)
        analysis_layout.addWidget(self.analyze_btn)
        
        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        analysis_layout.addWidget(self.progress_bar)
        
        # Reynolds number display
        self.reynolds_label = QLabel("Reynolds Number: -")
        analysis_layout.addWidget(self.reynolds_label)
        
        analysis_group.setLayout(analysis_layout)
        control_layout.addWidget(analysis_group)
        
        # ---- Visualization Controls Group ----
        viz_group = QGroupBox("Visualization Controls")
        viz_layout = QVBoxLayout()
        
        # View options in a form layout
        viz_options = QFormLayout()
        
        # Show streamlines checkbox
        self.streamlines_check = QCheckBox()
        self.streamlines_check.setChecked(False)
        self.streamlines_check.stateChanged.connect(self.toggle_streamlines)
        self.streamlines_check.setEnabled(False)
        viz_options.addRow("Show Streamlines:", self.streamlines_check)
        
        # Show coordinate axes checkbox
        self.axes_check = QCheckBox()
        self.axes_check.setChecked(True)
        self.axes_check.stateChanged.connect(self.toggle_axes)
        viz_options.addRow("Show Coordinate Axes:", self.axes_check)
        
        # Add view options to visualization layout
        viz_layout.addLayout(viz_options)
        
        # Add rotation controls
        rotation_layout = QHBoxLayout()
        rotate_left_btn = QPushButton("⟲ Rotate Left")
        rotate_left_btn.clicked.connect(self.rotate_model_left)
        rotation_layout.addWidget(rotate_left_btn)
        
        rotate_right_btn = QPushButton("⟳ Rotate Right")
        rotate_right_btn.clicked.connect(self.rotate_model_right)
        rotation_layout.addWidget(rotate_right_btn)
        
        viz_layout.addLayout(rotation_layout)
        
        # View controls (reset camera, etc.)
        reset_view_btn = QPushButton("Reset View")
        reset_view_btn.clicked.connect(self.reset_view)
        viz_layout.addWidget(reset_view_btn)
        
        viz_group.setLayout(viz_layout)
        control_layout.addWidget(viz_group)
        
        # Add a matplotlib canvas for the pressure scale
        self.plot_figure = Figure(figsize=(5, 2.5), dpi=100)  # Increased height
        self.plot_canvas = FigureCanvas(self.plot_figure)
        self.plot_canvas.setMinimumHeight(180)  # Increased minimum height
        control_layout.addWidget(self.plot_canvas)
        
        # Add result info panel
        info_group = QGroupBox("Results Info")
        info_layout = QVBoxLayout()
        self.info_label = QLabel("No analysis results yet")
        info_layout.addWidget(self.info_label)
        info_group.setLayout(info_layout)
        control_layout.addWidget(info_group)
        
        # Add a spacer at the bottom
        control_layout.addStretch()
        
        # Add the control panel and 3D view to the main layout
        main_layout.addWidget(control_panel)
        main_layout.addWidget(vtk_widget, 1)
        
        # Set up a timer for animation
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self.update_animation)
        
        # Initialize the VTK interactor
        if vtk_qt_available:
            self.iren.Initialize()
        
    def add_ground_plane(self):
        """Add a ground plane to the scene"""
        if not hasattr(self, 'renderer') or not self.renderer:
            return
            
        # Create a ground plane
        plane_source = vtk.vtkPlaneSource()
        plane_source.SetOrigin(-10, -10, 0)
        plane_source.SetPoint1(10, -10, 0)
        plane_source.SetPoint2(-10, 10, 0)
        plane_source.SetResolution(20, 20)
        plane_source.Update()
        
        # Add texture coordinates
        plane_tcoords = vtk.vtkTextureMapToPlane()
        plane_tcoords.SetInputConnection(plane_source.GetOutputPort())
        plane_tcoords.Update()
        
        # Create a mapper and actor
        plane_mapper = vtk.vtkPolyDataMapper()
        plane_mapper.SetInputConnection(plane_tcoords.GetOutputPort())
        
        # Create a checkerboard texture for the ground
        texture = vtk.vtkImageData()
        texture.SetDimensions(512, 512, 1)
        texture.AllocateScalars(vtk.VTK_UNSIGNED_CHAR, 3)
        
        # Fill the texture with a grid pattern
        for i in range(512):
            for j in range(512):
                color = 200 if ((i // 32) + (j // 32)) % 2 else 50
                texture.SetScalarComponentFromDouble(i, j, 0, 0, color)
                texture.SetScalarComponentFromDouble(i, j, 0, 1, color)
                texture.SetScalarComponentFromDouble(i, j, 0, 2, color)
        
        vtk_texture = vtk.vtkTexture()
        vtk_texture.SetInputData(texture)
        vtk_texture.InterpolateOff()
        
        # Create the ground actor
        self.ground_actor = vtk.vtkActor()
        self.ground_actor.SetMapper(plane_mapper)
        self.ground_actor.SetTexture(vtk_texture)
        
        # Add to the renderer
        self.renderer.AddActor(self.ground_actor)
        
    def import_model(self):
        """Import a 3D model from a file"""
        if not hasattr(self, 'renderer') or not self.renderer:
            QMessageBox.warning(self, "Feature Unavailable", 
                               "3D visualization is not available due to missing VTK Qt integration.")
            return
            
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Import 3D Model", "", 
            "3D Models (*.stl *.obj);;STL Files (*.stl);;OBJ Files (*.obj)",
            options=options
        )
        
        if file_path:
            # Create progress dialog for model loading
            progress = QProgressBar(self)
            progress.setWindowTitle("Loading Model")
            progress.setMinimum(0)
            progress.setMaximum(100)
            progress.setValue(0)
            progress.setWindowModality(Qt.WindowModal)
            progress.setGeometry(300, 300, 400, 50)
            progress.show()
            
            try:
                # Update progress - Starting load
                progress.setValue(10)
                QApplication.processEvents()
                
                # Load the mesh using trimesh
                mesh = trimesh.load(file_path)
                
                # Update progress - Mesh loaded
                progress.setValue(30)
                QApplication.processEvents()
                
                # Center the mesh at origin and scale it appropriately
                mesh.vertices -= mesh.center_mass
                
                # Calculate largest dimension for scaling
                max_dim = max(mesh.extents)
                
                # Scale to make the largest dimension 3 units
                scale_factor = 3.0 / max_dim
                mesh.vertices *= scale_factor
                
                # Move the bottom of the mesh to z=0 (ground level)
                min_z = min(mesh.vertices[:, 2])
                mesh.vertices[:, 2] -= min_z
                
                # Update progress - Mesh scaled
                progress.setValue(50)
                QApplication.processEvents()
                
                # Update the CFD solver with the new mesh
                self.cfd_solver.set_mesh(mesh)
                
                # Update progress - Preparing visualization
                progress.setValue(70)
                QApplication.processEvents()
                
                # Convert to VTK for visualization
                self.display_model(mesh)
                
                # Update progress - Almost done
                progress.setValue(90)
                QApplication.processEvents()
                
                # Enable analysis button
                self.analyze_btn.setEnabled(True)
                self.pressure_mapped = False
                self.streamlines_visible = False
                self.streamlines_check.setChecked(False)
                self.streamlines_check.setEnabled(False)
                
                # Update flow parameters
                self.update_airspeed()
                self.update_angle()
                self.update_yaw_from_slider()
                self.update_density()
                
                # Update Reynolds number display
                self.update_reynolds_display()
                
                # Update info
                self.info_label.setText(f"Model loaded: {os.path.basename(file_path)}\n"
                                       f"Triangles: {len(mesh.faces)}\n"
                                       f"Vertices: {len(mesh.vertices)}")
                
                # Update progress - Complete
                progress.setValue(100)
                
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load model: {str(e)}")
                print(f"Error loading model: {e}")
            
            finally:
                # Close progress dialog
                progress.close()
    
    def display_model(self, mesh):
        """Display the model in the VTK renderer"""
        if not hasattr(self, 'renderer') or not self.renderer:
            return
            
        # Remove existing model if any
        if self.model_actor:
            self.renderer.RemoveActor(self.model_actor)
        
        # Clean up streamlines if any
        if self.streamlines_actor:
            self.renderer.RemoveActor(self.streamlines_actor)
            self.streamlines_actor = None
        
        # Create VTK polydata from trimesh
        vtk_points = vtk.vtkPoints()
        vtk_cells = vtk.vtkCellArray()
        
        # Add points
        for vertex in mesh.vertices:
            vtk_points.InsertNextPoint(vertex)
        
        # Add triangular faces
        for face in mesh.faces:
            vtk_cells.InsertNextCell(3)  # Triangle has 3 points
            for vertex_index in face:
                vtk_cells.InsertCellPoint(vertex_index)
        
        # Create polydata
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(vtk_points)
        polydata.SetPolys(vtk_cells)
        
        # Compute normals
        normals = vtk.vtkPolyDataNormals()
        normals.SetInputData(polydata)
        normals.SetFeatureAngle(80)
        normals.AutoOrientNormalsOn()
        normals.ComputePointNormalsOn()
        normals.ComputeCellNormalsOn()
        normals.SplittingOff()
        normals.ConsistencyOn()
        normals.Update()
        
        # Create mapper
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(normals.GetOutput())
        
        # Create actor
        self.model_actor = vtk.vtkActor()
        self.model_actor.SetMapper(mapper)
        self.model_actor.GetProperty().SetColor(0.8, 0.8, 0.9)
        
        # Add to renderer
        self.renderer.AddActor(self.model_actor)
        
        # Reset the camera
        self.renderer.ResetCamera()
        self.vtk_widget.GetRenderWindow().Render()
    
    def update_airspeed(self):
        """Update the airspeed in the CFD solver"""
        if self.cfd_solver:
            airspeed = self.airspeed_spin.value()
            # Handle zero or near-zero airspeed
            if airspeed < 0.01:
                self.reynolds_label.setText("Reynolds Number: 0.00")
                if self.pressure_mapped and self.model_actor:
                    # Reset to uniform color for zero airspeed
                    self.model_actor.GetProperty().SetColor(0.8, 0.8, 0.9)
                    self.vtk_widget.GetRenderWindow().Render()
                    self.info_label.setText("Pressure Results:\nAirspeed is zero - no pressure variation")
            else:
                self.cfd_solver.set_airspeed(airspeed)
                self.update_reynolds_display()
                
                # Update pressure visualization if pressure is already mapped
                if self.pressure_mapped and self.model_actor:
                    self.display_pressure(auto_scale=True)
    
    def update_angle(self):
        """Update the angle of attack in the CFD solver"""
        if self.cfd_solver:
            self.cfd_solver.set_angle(self.angle_spin.value())
                
            # Update pressure visualization if already mapped
            if self.pressure_mapped and self.model_actor:
                self.display_pressure(auto_scale=False)
    
    def update_yaw_from_slider(self):
        """Update the yaw angle from the slider value"""
        yaw_value = self.yaw_slider.value()
        self.yaw_label.setText(f"{yaw_value}°")
        if self.cfd_solver:
            self.cfd_solver.set_yaw(yaw_value)
                
            # Update pressure visualization if already mapped
            if self.pressure_mapped and self.model_actor:
                self.display_pressure(auto_scale=False)
    
    def update_density(self):
        """Update the air density in the CFD solver"""
        if self.cfd_solver:
            self.cfd_solver.set_density(self.density_spin.value())
            self.update_reynolds_display()
            
    def update_reynolds_display(self):
        """Update the Reynolds number display"""
        if self.cfd_solver and self.cfd_solver.mesh is not None:
            reynolds = self.cfd_solver.calculate_reynolds_number()
            self.reynolds_label.setText(f"Reynolds Number: {reynolds:.2e}")
    
    def run_cfd_analysis(self):
        """Run the CFD analysis with progress simulation and robust error handling"""
        if not self.model_actor or not self.cfd_solver.mesh or self.analysis_running:
            return
            
        try:
            self.analysis_running = True
            self.analyze_btn.setEnabled(False)
            self.progress_bar.setValue(0)
            
            # Step 1: Calculate pressure (0-40%)
            for i in range(41):
                self.progress_bar.setValue(i)
                QApplication.processEvents()
                if i == 40:
                    try:
                        self.display_pressure()
                    except Exception as e:
                        print(f"Error in pressure calculation: {str(e)}")
                        QMessageBox.warning(self, "Analysis Warning", 
                                          f"Pressure calculation encountered an issue: {str(e)}")
            
            # Step 2: Generate streamlines (40-95%)
            self.progress_bar.setValue(41)
            QApplication.processEvents()
            
            try:
                # Generate streamlines
                streamlines, colors = self.cfd_solver.generate_streamlines(domain_size=10, grid_density=6)
                
                # Update progress in chunks
                for i in range(41, 95, 5):
                    self.progress_bar.setValue(i)
                    QApplication.processEvents()
                    time.sleep(0.05)  # Small delay to show progress
                
                # Create streamline actors
                self.create_streamline_actors(streamlines, colors)
                self.streamlines_check.setEnabled(True)
                
            except Exception as e:
                print(f"Error in streamline generation: {str(e)}")
                QMessageBox.warning(self, "Analysis Warning", 
                                   f"Streamline generation encountered an issue: {str(e)}")
            
            # Step 3: Finish analysis (95-100%)
            for i in range(95, 101):
                self.progress_bar.setValue(i)
                QApplication.processEvents()
                time.sleep(0.02)
                
            # Finish analysis
            self.analysis_running = False
            self.analyze_btn.setEnabled(True)
            QMessageBox.information(self, "Analysis Complete", 
                                  "CFD analysis has been completed successfully.")
                                  
        except Exception as e:
            # Handle any unexpected errors
            self.analysis_running = False
            self.analyze_btn.setEnabled(True)
            print(f"Error during CFD analysis: {str(e)}")
            QMessageBox.critical(self, "Analysis Error", 
                               f"CFD analysis failed: {str(e)}")
    
    def display_pressure(self, auto_scale=False):
        """Calculate and display pressure on the model surface with robust error handling
        
        Args:
            auto_scale (bool): If True, scale the pressure range automatically
                              based on the current airspeed
        """
        if not self.model_actor or not self.cfd_solver.mesh or not hasattr(self, 'renderer') or not self.renderer:
            return
            
        try:
            # Handle zero airspeed case specially
            if self.cfd_solver.airspeed <= 0.01:
                if hasattr(self, 'model_actor') and self.model_actor:
                    # Set uniform color for zero airspeed
                    self.model_actor.GetProperty().SetColor(0.8, 0.8, 0.9)
                    self.vtk_widget.GetRenderWindow().Render()
                    self.info_label.setText("Pressure Results:\nAirspeed is zero - no pressure variation")
                    return
            
            # Calculate pressure distribution
            pressures = self.cfd_solver.calculate_pressure()
            
            if pressures is None or len(pressures) == 0:
                print("Warning: No pressure data calculated")
                return
            
            # Get the polydata from the mapper
            polydata = self.model_actor.GetMapper().GetInput()
            
            # Create color array for pressure visualization
            colors = vtk.vtkUnsignedCharArray()
            colors.SetNumberOfComponents(3)
            colors.SetName("Colors")
            
            # Create a color lookup table
            lut = vtk.vtkLookupTable()
            
            # Calculate appropriate pressure range based on actual pressures
            actual_min = float(np.min(pressures))
            actual_max = float(np.max(pressures))
            
            # Protect against NaN or infinite values
            if np.isnan(actual_min) or np.isnan(actual_max) or np.isinf(actual_min) or np.isinf(actual_max):
                print(f"Warning: Invalid pressure values detected. Min: {actual_min}, Max: {actual_max}")
                # Set to safe values
                actual_min = -1.0
                actual_max = 1.0
                
            # Ensure we don't have identical min and max (would cause division by zero)
            if abs(actual_max - actual_min) < 1e-6:
                actual_max = actual_min + 1.0
                
            pressure_range = max(abs(actual_min), abs(actual_max))
            
            # Create a more balanced and realistic pressure scale
            # This ensures the color range is appropriate for the values
            pressure_min = -pressure_range
            pressure_max = pressure_range
            
            # Use a refined blue-to-red color map for better visualization
            lut.SetTableRange(pressure_min, pressure_max)
            lut.SetHueRange(0.667, 0.0)  
            lut.SetSaturationRange(0.8, 0.8)
            lut.SetValueRange(0.9, 0.9)
            lut.Build()
            
            # Map pressures to colors directly
            for pressure in pressures:
                # Handle NaN or infinite values
                if np.isnan(pressure) or np.isinf(pressure):
                    pressure = 0
                    
                rgb = [0, 0, 0]
                lut.GetColor(pressure, rgb)
                colors.InsertNextTuple3(
                    int(rgb[0] * 255), 
                    int(rgb[1] * 255), 
                    int(rgb[2] * 255)
                )
            
            # Assign colors to polydata
            polydata.GetCellData().SetScalars(colors)
            
            # Update the mapper
            self.model_actor.GetMapper().Update()
            
            # Render
            self.vtk_widget.GetRenderWindow().Render()
            
            # Mark pressure as mapped
            self.pressure_mapped = True
            
            # Update the pressure scale visualization
            try:
                self.update_pressure_scale(pressure_min, pressure_max, lut)
            except Exception as e:
                print(f"Error updating pressure scale: {str(e)}")
            
            # Update info label
            pressure_avg = np.mean(pressures)
            pressure_min_actual = np.min(pressures)
            pressure_max_actual = np.max(pressures)
            
            self.info_label.setText(f"Pressure Results:\n"
                                   f"Min: {pressure_min_actual:.2f} Pa\n"
                                   f"Max: {pressure_max_actual:.2f} Pa\n"
                                   f"Avg: {pressure_avg:.2f} Pa")
                                   
        except Exception as e:
            print(f"Error in display_pressure: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def update_pressure_scale(self, min_p, max_p, lut):
        """Update the pressure scale visualization with consistent number of ticks and labels"""
        try:
            # Check for invalid input values
            if np.isnan(min_p) or np.isnan(max_p) or np.isinf(min_p) or np.isinf(max_p):
                print(f"Warning: Invalid pressure range for scale. Min: {min_p}, Max: {max_p}")
                min_p = -1.0
                max_p = 1.0
                
            # Ensure min < max
            if min_p >= max_p:
                min_p = -1.0
                max_p = 1.0
            
            # Clear the figure
            self.plot_figure.clear()
            
            # Add an axes with more margin for better visibility
            ax = self.plot_figure.add_subplot(111)
            
            # Create a colorbar with high resolution
            gradient = np.linspace(0, 1, 512)  # More points for smoother gradient
            gradient = np.vstack((gradient, gradient))
            
            # Get colors from the LUT with high resolution
            colors = []
            for i in range(512):
                val = min_p + (max_p - min_p) * (i / 511.0)
                rgb = [0, 0, 0]
                lut.GetColor(val, rgb)
                colors.append(rgb)
            
            # Plot the colorbar with larger size
            img = ax.imshow(gradient, aspect='auto', cmap=plt.cm.colors.ListedColormap(colors))
            
            # Remove y axis ticks
            ax.set_yticks([])
            
            # Create 5 evenly spaced tick positions - ALWAYS use 5 ticks
            positions = [0, 128, 256, 384, 511]
            ax.set_xticks(positions)
            
            # Create 5 evenly spaced values for the labels
            values = [
                min_p,
                min_p + 0.25 * (max_p - min_p),
                min_p + 0.5 * (max_p - min_p),
                min_p + 0.75 * (max_p - min_p),
                max_p
            ]
            
            # Format the values according to their magnitude
            if abs(min_p) > 5000 or abs(max_p) > 5000:
                # Use scientific notation for large numbers
                labels = [f"{v:.1e}" for v in values]
            else:
                # Use fixed notation for smaller numbers
                labels = [f"{v:.1f}" for v in values]
                
            # Set the labels to the ticks
            ax.set_xticklabels(labels)
            
            # Add a vertical line at zero if it's in the range
            if min_p < 0 and max_p > 0:
                zero_pos = (0 - min_p) / (max_p - min_p) * 511
                ax.axvline(x=zero_pos, color='white', linestyle='-', linewidth=1.0, alpha=0.7)
            
            # Set labels with larger font sizes
            ax.set_xlabel("Pressure (Pa)", fontsize=12)
            ax.set_title("Pressure Distribution", fontsize=14, fontweight='bold')
            
            # Add color labels for clarity
            ax.text(20, gradient.shape[0]*0.5, "LOW", color='white', 
                    fontsize=10, fontweight='bold', ha='left', va='center')
            ax.text(490, gradient.shape[0]*0.5, "HIGH", color='white', 
                    fontsize=10, fontweight='bold', ha='right', va='center')
            
            # Increase tick label size
            ax.tick_params(axis='x', labelsize=10)
            
            # Add more padding
            self.plot_figure.tight_layout(pad=1.5)
            
            # Update the canvas
            self.plot_canvas.draw()
            
        except Exception as e:
            print(f"Error in update_pressure_scale: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def generate_streamlines(self):
        """Generate streamlines around the model - returns streamlines and colors"""
        if not self.model_actor or not self.cfd_solver.mesh or not hasattr(self, 'renderer') or not self.renderer:
            return None, None
            
        # Generate streamlines from CFD solver
        return self.cfd_solver.generate_streamlines(domain_size=10, grid_density=6)
    
    def create_streamline_actors(self, streamlines, colors):
        """Create VTK actors for streamlines
        
        This method is separated from the calculation to improve UI responsiveness
        """
        if not self.model_actor or not hasattr(self, 'renderer') or not self.renderer:
            return
            
        # Clear existing streamlines
        if self.streamlines_actor:
            self.renderer.RemoveActor(self.streamlines_actor)
            self.streamlines_actor = None
        
        if streamlines is None:
            return
            
        # Create a VTK actor for all streamlines
        streamlines_polydata = vtk.vtkAppendPolyData()
        
        # Create individual streamline actors
        for i, points in enumerate(streamlines):
            if len(points) < 2:
                continue
                
            # Create a VTK points object
            vtk_points = vtk.vtkPoints()
            for point in points:
                vtk_points.InsertNextPoint(point)
                
            # Create a VTK line
            vtk_line = vtk.vtkPolyLine()
            vtk_line.GetPointIds().SetNumberOfIds(len(points))
            for j in range(len(points)):
                vtk_line.GetPointIds().SetId(j, j)
                
            # Create cells
            vtk_cells = vtk.vtkCellArray()
            vtk_cells.InsertNextCell(vtk_line)
            
            # Create a polydata object
            line_polydata = vtk.vtkPolyData()
            line_polydata.SetPoints(vtk_points)
            line_polydata.SetLines(vtk_cells)
            
            # Set color
            vtk_colors = vtk.vtkUnsignedCharArray()
            vtk_colors.SetNumberOfComponents(3)
            vtk_colors.SetName("Colors")
            
            r, g, b = colors[i]
            for _ in range(len(points)):
                vtk_colors.InsertNextTuple3(
                    int(r * 255), 
                    int(g * 255), 
                    int(b * 255)
                )
                
            line_polydata.GetPointData().SetScalars(vtk_colors)
            
            # Add to the append filter
            streamlines_polydata.AddInputData(line_polydata)
            
        # Update the append filter
        streamlines_polydata.Update()
        
        # Create mapper and actor
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(streamlines_polydata.GetOutputPort())
        
        self.streamlines_actor = vtk.vtkActor()
        self.streamlines_actor.SetMapper(mapper)
        self.streamlines_actor.GetProperty().SetLineWidth(2)
        
        # Add to renderer but don't show by default
        self.renderer.AddActor(self.streamlines_actor)
        self.streamlines_actor.SetVisibility(False)
        
        # Render
        self.vtk_widget.GetRenderWindow().Render()
    
    def toggle_streamlines(self):
        """Toggle visibility of streamlines"""
        if self.streamlines_actor and hasattr(self, 'vtk_widget'):
            self.streamlines_visible = self.streamlines_check.isChecked()
            self.streamlines_actor.SetVisibility(self.streamlines_visible)
            self.vtk_widget.GetRenderWindow().Render()
    
    def toggle_axes(self):
        """Toggle visibility of coordinate axes"""
        if hasattr(self, 'axes') and hasattr(self, 'vtk_widget'):
            self.axes.SetVisibility(self.axes_check.isChecked())
            self.vtk_widget.GetRenderWindow().Render()
    
    def rotate_model_left(self):
        """Rotate the model 15 degrees left (counterclockwise)"""
        if self.model_actor and hasattr(self, 'vtk_widget'):
            self.model_actor.RotateZ(15)
            self.vtk_widget.GetRenderWindow().Render()
            
    def rotate_model_right(self):
        """Rotate the model 15 degrees right (clockwise)"""
        if self.model_actor and hasattr(self, 'vtk_widget'):
            self.model_actor.RotateZ(-15)
            self.vtk_widget.GetRenderWindow().Render()
    
    def reset_view(self):
        """Reset the camera view"""
        if not hasattr(self, 'camera') or not hasattr(self, 'renderer') or not hasattr(self, 'vtk_widget'):
            return
            
        self.camera.SetPosition(0, -10, 3)
        self.camera.SetFocalPoint(0, 0, 0)
        self.camera.SetViewUp(0, 0, 1)
        self.renderer.ResetCamera()
        self.vtk_widget.GetRenderWindow().Render()
    
    def update_animation(self):
        """Update the animation for each timer tick"""
        if self.streamlines_visible and self.streamlines_actor and hasattr(self, 'vtk_widget'):
            # Rotate streamlines slightly for animation effect
            self.streamlines_actor.RotateZ(0.5)
            self.vtk_widget.GetRenderWindow().Render()
            
    def take_screenshot(self):
        """Take a screenshot of the current view"""
        if not hasattr(self, 'vtk_widget'):
            QMessageBox.warning(self, "Feature Unavailable", 
                               "Screenshot feature is not available without VTK.")
            return
            
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Screenshot", "", 
            "PNG Files (*.png);;All Files (*)",
            options=options
        )
        
        if file_path:
            try:
                # Get the render window
                window = self.vtk_widget.GetRenderWindow()
                
                # Create window to image filter
                window_to_image = vtk.vtkWindowToImageFilter()
                window_to_image.SetInput(window)
                window_to_image.SetInputBufferTypeToRGBA()
                window_to_image.ReadFrontBufferOff()
                window_to_image.Update()
                
                # Write to file
                writer = vtk.vtkPNGWriter()
                writer.SetFileName(file_path)
                writer.SetInputConnection(window_to_image.GetOutputPort())
                writer.Write()
                
                QMessageBox.information(self, "Screenshot Saved", 
                                      f"Screenshot saved to {file_path}")
            except Exception as e:
                QMessageBox.critical(self, "Screenshot Error", 
                                   f"Failed to save screenshot: {str(e)}")


if __name__ == "__main__":
    # Create QApplication
    app = QApplication(sys.argv)
    
    # Create and show the main window
    window = CFDVisualizationApp()
    window.show()
    
    # Run the application
    sys.exit(app.exec_())