"""Offscreen VTK rendering directly from tessellated canonical B-reps."""
from pathlib import Path
import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray

COLORS={'Frame':(0.15,.48,.57),'Guard':(.88,.61,.22),'Bottom':(.2,.39,.48),'TPU':(.24,.27,.33),
        'Motor':(.28,.30,.35),'Prop':(.33,.82,.87),'Battery':(.62,.27,.29),'FC':(.18,.64,.42),
        'ELRS':(.65,.38,.7),'Camera':(.28,.29,.38),'MTF02P':(.80,.28,.52),'Standoff':(.7,.73,.77),'Isolator':(.66,.69,.7)}

def color(name):
    for prefix,c in COLORS.items():
        if name.startswith(prefix): return c
    return COLORS['TPU']

def actor(mesh,c,alpha=1):
    points=vtk.vtkPoints(); points.SetData(numpy_to_vtk(np.asarray(mesh.vertices,dtype=np.float64),deep=True))
    cells=vtk.vtkCellArray()
    faces=np.column_stack([np.full(len(mesh.faces),3),mesh.faces]).astype(np.int64)
    cells.SetCells(len(faces),numpy_to_vtkIdTypeArray(faces.ravel(),deep=True))
    poly=vtk.vtkPolyData(); poly.SetPoints(points); poly.SetPolys(cells)
    normals=vtk.vtkPolyDataNormals(); normals.SetInputData(poly); normals.SetFeatureAngle(55); normals.SplittingOn(); normals.Update()
    mapper=vtk.vtkPolyDataMapper(); mapper.SetInputConnection(normals.GetOutputPort())
    a=vtk.vtkActor(); a.SetMapper(mapper); a.GetProperty().SetColor(*c); a.GetProperty().SetOpacity(alpha)
    a.GetProperty().SetSpecular(.22); a.GetProperty().SetSpecularPower(28)
    return a

def render(meshes,path,title,view='iso',hardware=None,explode=False,diagnostics=None):
    ren=vtk.vtkRenderer(); ren.SetBackground(.94,.96,.97)
    window=vtk.vtkRenderWindow(); window.SetOffScreenRendering(1); window.SetSize(1600,1200); window.AddRenderer(ren)
    window.SetMultiSamples(4)
    for name,m in meshes.items():
        a=actor(m,color(name))
        if explode:
            if name.startswith('Guard'): a.SetPosition(0,0,40)
            if name.startswith('Bottom') or name.startswith('Foot') or name.startswith('BatteryPad'): a.SetPosition(0,0,-25)
            if name.startswith('CameraCradle'): a.SetPosition(35,0,12)
        ren.AddActor(a)
    if hardware:
        for name,m in hardware.items(): ren.AddActor(actor(m,color(name),.24 if name.startswith('Prop') else .78))
    if diagnostics:
        for name,m in diagnostics.items(): ren.AddActor(actor(m,(.85,.25,.50),.12))
    positions={'iso':(260,-330,240),'top':(0,0,500),'side':(0,-500,0),'front':(500,0,0),'underside':(240,-300,-260)}
    cam=ren.GetActiveCamera(); cam.SetPosition(*positions.get(view,positions['iso'])); cam.SetFocalPoint(0,0,0)
    cam.SetViewUp(0,1,0) if view=='top' else cam.SetViewUp(0,0,1)
    cam.ParallelProjectionOn(); ren.ResetCamera(); cam.Zoom(.94); ren.ResetCameraClippingRange()
    label=vtk.vtkTextActor(); label.SetInput(title+'\nCDE1302 / ASTRA 01   |   millimeters   |   prototype')
    label.SetPosition(36,1130); prop=label.GetTextProperty(); prop.SetColor(.12,.20,.25); prop.SetFontSize(24); prop.SetFontFamilyToArial(); ren.AddActor2D(label)
    foot=vtk.vtkTextActor(); foot.SetInput('x forward   y right   CAD z up    /    teal PETG frame   amber replaceable guards   charcoal TPU')
    foot.SetPosition(36,30); foot.GetTextProperty().SetColor(.25,.31,.35); foot.GetTextProperty().SetFontSize(17); ren.AddActor2D(foot)
    window.Render()
    grab=vtk.vtkWindowToImageFilter(); grab.SetInput(window); grab.SetInputBufferTypeToRGB(); grab.ReadFrontBufferOff(); grab.Update()
    writer=vtk.vtkPNGWriter(); writer.SetFileName(str(path)); writer.SetInputConnection(grab.GetOutputPort()); writer.Write(); window.Finalize()

def topology_render(candidates,path):
    from PIL import Image,ImageDraw
    files=[]
    for name,m in candidates.items():
        f=Path(path).with_name('candidate_'+name+'.png'); render({'Frame':m},f,name+' / representative arm',view='iso'); files.append(f)
    ims=[Image.open(f).resize((800,600)) for f in files]
    canvas=Image.new('RGB',(2400,600),'white')
    for i,im in enumerate(ims): canvas.paste(im,(800*i,0))
    canvas.save(path)
