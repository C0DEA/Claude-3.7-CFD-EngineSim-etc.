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
        Simplified CFD calculation - This is not a full Navier-Stokes solver!
        
        For demonstration purposes, we use a simplified approach that combines:
        1. Surface orientation relative to flow direction
        2. Distance effects from leading edge
        3. Basic pressure coefficient estimation
        
        A real CFD solver would:
        - Mesh the entire flow domain (not just the surface)
        - Solve the discretized Navier-Stokes equations
        - Account for viscous effects, turbulence, boundary layers, etc.
        """
        if self.mesh is None:
            return None
            
        # Get mesh face normals
        face_normals = self.mesh.face_normals
        
        # Create a flow direction vector based on angle of attack and yaw
        # First create vector based on pitch (angle of attack)
        pitch_rad = np.radians(self.angle)
        yaw_rad = np.radians(self.yaw)
        
        # Calculate direction components
        x = np.cos(pitch_rad) * np.cos(yaw_rad)
        y = np.cos(pitch_rad) * np.sin(yaw_rad)
        z = np.sin(pitch_rad)
        
        flow_dir = np.array([x, y, z])
        
        # Calculate dot product between face normals and flow direction
        # This gives a simplified approximation of pressure based on surface orientation
        dots = np.dot(face_normals, flow_dir)
        
        # Dynamic pressure
        dyn_pressure = 0.5 * self.density * self.airspeed**2
        
        # Calculate a simplified pressure coefficient
        # Faces directly facing the flow (dots close to -1) get high pressure
        # Faces parallel or away from flow get lower pressure
        cp = 1.0 + dots  # This is a simplification of the pressure coefficient
        
        # Add some variation based on position (very simplified)
        face_centers = self.mesh.triangles_center
        x_position = face_centers[:, 0]
        x_range = np.max(x_position) - np.min(x_position)
        position_factor = (x_position - np.min(x_position)) / x_range if x_range > 0 else 0.5
        
        # Combine position and orientation effects
        # Leading edge faces (low position_factor) get higher pressure
        pressures = dyn_pressure * (cp - 0.3 * position_factor)
        
        return pressures
    
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
            
            # Add a 3D arrow to show flow direction
            self.add_flow_direction_indicator()
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
        
        # Export results button
        export_btn = QPushButton("Export Pressure Results")
        export_btn.clicked.connect(self.export_results)
        export_btn.setEnabled(False)
        self.export_btn = export_btn
        file_layout.addWidget(export_btn)
        
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
        self.airspeed_spin.setRange(1.0, 500.0)
        self.airspeed_spin.setValue(10.0)
        self.airspeed_spin.setSuffix(" m/s")
        self.airspeed_spin.valueChanged.connect(self.update_airspeed)
        flow_layout.addRow("Airspeed:", self.airspeed_spin)
        
        # Angle of attack input (pitch)
        self.angle_spin = QDoubleSpinBox()
        self.angle_spin.setRange(-90.0, 90.0)
        self.angle_spin.setValue(0.0)
        self.angle_spin.setSuffix("°")
        self.angle_spin.valueChanged.connect(self.update_angle)
        flow_layout.addRow("Angle of Attack (Pitch):", self.angle_spin)
        
        # Yaw angle input (horizontal rotation)
        self.yaw_spin = QDoubleSpinBox()
        self.yaw_spin.setRange(-180.0, 180.0)
        self.yaw_spin.setValue(0.0)
        self.yaw_spin.setSuffix("°")
        self.yaw_spin.valueChanged.connect(self.update_yaw)
        flow_layout.addRow("Flow Direction (Yaw):", self.yaw_spin)
        
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
        self.plot_figure = Figure(figsize=(5, 1.5), dpi=100)
        self.plot_canvas = FigureCanvas(self.plot_figure)
        self.plot_canvas.setMinimumHeight(120)
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
        
    def add_flow_direction_indicator(self):
        """Add a 3D arrow to visualize the flow direction"""
        if not hasattr(self, 'renderer') or not self.renderer:
            return
            
        # Create arrow source
        arrow = vtk.vtkArrowSource()
        arrow.SetTipResolution(20)
        arrow.SetShaftResolution(20)
        arrow.SetTipRadius(0.2)
        arrow.SetShaftRadius(0.05)
        
        # Transform to position and orient the arrow
        transform = vtk.vtkTransform()
        transform.Translate(-5, 0, 1)  # Position in front of the scene
        transform.RotateZ(180)  # Point in the positive X direction
        
        # Apply the transform
        transformFilter = vtk.vtkTransformPolyDataFilter()
        transformFilter.SetInputConnection(arrow.GetOutputPort())
        transformFilter.SetTransform(transform)
        transformFilter.Update()
        
        # Create mapper and actor
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(transformFilter.GetOutputPort())
        
        self.flow_arrow = vtk.vtkActor()
        self.flow_arrow.SetMapper(mapper)
        self.flow_arrow.GetProperty().SetColor(1, 0, 0)  # Red arrow
        
        # Add to renderer
        self.renderer.AddActor(self.flow_arrow)
        
    def update_flow_direction_arrow(self):
        """Update the flow direction arrow based on current pitch and yaw angles"""
        if not hasattr(self, 'flow_arrow') or not hasattr(self, 'renderer') or not self.renderer:
            return
            
        # Get current angles
        pitch = self.angle_spin.value() if hasattr(self, 'angle_spin') else 0
        yaw = self.yaw_spin.value() if hasattr(self, 'yaw_spin') else 0
        
        # Create a new transform
        transform = vtk.vtkTransform()
        transform.Identity()
        transform.Translate(-5, 0, 1)  # Position in front of the scene
        
        # Apply rotations
        transform.RotateZ(yaw)     # Yaw rotation (around Z)
        transform.RotateY(-pitch)  # Pitch rotation (around Y)
        transform.RotateZ(180)     # Base rotation to point in positive X
        
        # Set the transform to the actor
        self.flow_arrow.SetUserTransform(transform)
        
        # Update the rendering
        if hasattr(self, 'vtk_widget'):
            self.vtk_widget.GetRenderWindow().Render()
        
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
            self.current_model_path = file_path
            try:
                # Load the mesh using trimesh
                mesh = trimesh.load(file_path)
                
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
                
                # Update the CFD solver with the new mesh
                self.cfd_solver.set_mesh(mesh)
                
                # Convert to VTK for visualization
                self.display_model(mesh)
                
                # Enable analysis button
                self.analyze_btn.setEnabled(True)
                self.pressure_mapped = False
                self.streamlines_visible = False
                self.streamlines_check.setChecked(False)
                self.streamlines_check.setEnabled(False)
                
                # Update flow parameters
                self.update_airspeed()
                self.update_angle()
                self.update_yaw()
                self.update_density()
                
                # Update Reynolds number display
                self.update_reynolds_display()
                
                # Update info
                self.info_label.setText(f"Model loaded: {os.path.basename(file_path)}\n"
                                       f"Triangles: {len(mesh.faces)}\n"
                                       f"Vertices: {len(mesh.vertices)}")
                
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load model: {str(e)}")
                print(f"Error loading model: {e}")
    
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
            self.cfd_solver.set_airspeed(self.airspeed_spin.value())
            self.update_reynolds_display()
    
    def update_angle(self):
        """Update the angle of attack in the CFD solver"""
        if self.cfd_solver:
            self.cfd_solver.set_angle(self.angle_spin.value())
            if hasattr(self, 'update_flow_direction_arrow'):
                self.update_flow_direction_arrow()
    
    def update_yaw(self):
        """Update the yaw angle in the CFD solver"""
        if self.cfd_solver:
            self.cfd_solver.set_yaw(self.yaw_spin.value())
            if hasattr(self, 'update_flow_direction_arrow'):
                self.update_flow_direction_arrow()
    
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
        """Run the CFD analysis with progress simulation"""
        if not self.model_actor or not self.cfd_solver.mesh or self.analysis_running:
            return
            
        self.analysis_running = True
        self.analyze_btn.setEnabled(False)
        self.progress_bar.setValue(0)
        
        # Simulate a multi-step analysis process
        for i in range(101):
            # Update progress bar
            self.progress_bar.setValue(i)
            
            # Process events to keep UI responsive
            QApplication.processEvents()
            
            # Simulate computation time
            time.sleep(0.02)
            
            # At certain milestones, update the visualization
            if i == 33:
                # Calculate pressure distribution
                self.display_pressure()
                
            if i == 66:
                # Generate streamlines
                self.generate_streamlines()
                self.streamlines_check.setEnabled(True)
                
            if i == 100:
                # Finish analysis
                self.analysis_running = False
                self.analyze_btn.setEnabled(True)
                self.export_btn.setEnabled(True)
                QMessageBox.information(self, "Analysis Complete", 
                                      "CFD analysis has been completed successfully.")
    
    def display_pressure(self):
        """Calculate and display pressure on the model surface"""
        if not self.model_actor or not self.cfd_solver.mesh or not hasattr(self, 'renderer') or not self.renderer:
            return
        
        # Calculate pressure distribution
        pressures = self.cfd_solver.calculate_pressure()
        
        if pressures is None:
            return
        
        # Get the polydata from the mapper
        polydata = self.model_actor.GetMapper().GetInput()
        
        # Create color array for pressure visualization
        colors = vtk.vtkUnsignedCharArray()
        colors.SetNumberOfComponents(3)
        colors.SetName("Colors")
        
        # Create a color lookup table
        lut = vtk.vtkLookupTable()
        lut.SetTableRange(np.min(pressures), np.max(pressures))
        lut.SetHueRange(0.667, 0.0)  # Blue to red
        lut.Build()
        
        # Map pressures to colors
        for pressure in pressures:
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
        self.update_pressure_scale(np.min(pressures), np.max(pressures), lut)
        
        # Update info label
        pressure_avg = np.mean(pressures)
        pressure_min = np.min(pressures)
        pressure_max = np.max(pressures)
        
        self.info_label.setText(f"Pressure Results:\n"
                               f"Min: {pressure_min:.2f} Pa\n"
                               f"Max: {pressure_max:.2f} Pa\n"
                               f"Avg: {pressure_avg:.2f} Pa")
    
    def update_pressure_scale(self, min_p, max_p, lut):
        """Update the pressure scale visualization"""
        # Clear the figure
        self.plot_figure.clear()
        
        # Add an axes
        ax = self.plot_figure.add_subplot(111)
        
        # Create a colorbar
        gradient = np.linspace(0, 1, 256)
        gradient = np.vstack((gradient, gradient))
        
        # Get colors from the LUT
        colors = []
        for i in range(256):
            val = min_p + (max_p - min_p) * (i / 255.0)
            rgb = [0, 0, 0]
            lut.GetColor(val, rgb)
            colors.append(rgb)
        
        # Plot the colorbar
        ax.imshow(gradient, aspect='auto', cmap=plt.cm.colors.ListedColormap(colors))
        
        # Remove y axis ticks
        ax.set_yticks([])
        
        # Set x axis ticks to pressure values
        ax.set_xticks([0, 127, 255])
        ax.set_xticklabels([f"{min_p:.2f}", f"{(min_p + max_p) / 2:.2f}", f"{max_p:.2f}"])
        
        # Set labels
        ax.set_xlabel("Pressure (Pa)")
        ax.set_title("Pressure Distribution")
        
        # Update the canvas
        self.plot_canvas.draw()
    
    def generate_streamlines(self):
        """Generate and visualize streamlines around the model"""
        if not self.model_actor or not self.cfd_solver.mesh or not hasattr(self, 'renderer') or not self.renderer:
            return
            
        # Clear existing streamlines
        if self.streamlines_actor:
            self.renderer.RemoveActor(self.streamlines_actor)
            self.streamlines_actor = None
            
        # Generate streamlines
        streamlines, colors = self.cfd_solver.generate_streamlines(domain_size=10, grid_density=6)
        
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
            
    def export_results(self):
        """Export the pressure results to a CSV file"""
        if not self.pressure_mapped or not self.cfd_solver.mesh:
            return
            
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Export Pressure Results", "", 
            "CSV Files (*.csv);;All Files (*)",
            options=options
        )
        
        if file_path:
            try:
                # Calculate pressure
                pressures = self.cfd_solver.calculate_pressure()
                face_centers = self.cfd_solver.mesh.triangles_center
                
                # Write to CSV
                with open(file_path, 'w') as f:
                    f.write("X,Y,Z,Pressure\n")
                    for i, center in enumerate(face_centers):
                        f.write(f"{center[0]},{center[1]},{center[2]},{pressures[i]}\n")
                        
                QMessageBox.information(self, "Export Successful", 
                                      f"Pressure data exported to {file_path}")
                                      
            except Exception as e:
                QMessageBox.critical(self, "Export Error", 
                                   f"Failed to export results: {str(e)}")
    
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