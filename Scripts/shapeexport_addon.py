bl_info = {
"name":"Shape Export Addon",
"author":"Timothy Costigan <costigt@tcd.ie>",
"version":(0,1),
"blender":(2,6,6),
"location":"",
"description":"Exports spheres as an xml for creating Bullet btMultiSphere shapes",
"warning":"",
"wiki_url":"",
"tracker_url":"",
"category":"Import-Export"}

import bpy
from bpy_extras.io_utils import ExportHelper
from xml.dom.minidom import Document

class ShapeExporter(bpy.types.Operator,ExportHelper):
    bl_idname = "export_shape.xml"
    bl_label = "Export to shape .xml"
    
    filename_ext = ".xml"
    
    def execute(self,context):
        filepath = self.filepath
        filepath = bpy.path.ensure_ext(filepath,self.filename_ext)
        
        #PERFORM ACTIONS HERE
        doc = Document()
        
        shape = doc.createElement("shape")
        
        spheres = bpy.context.selected_objects
        
        for sphere in spheres:
            part = doc.createElement("sphere")
            part.setAttribute("radius",str(sphere.dimensions.x/2.0))
            #flip axes to xz-y as per ogre standard
            part.setAttribute("x",str(sphere.location.x))
            part.setAttribute("y",str(sphere.location.z))
            part.setAttribute("z",str(-sphere.location.y))
            shape.appendChild(part)
            
        doc.appendChild(shape)
        
        file = open(filepath,"w")
        file.write(doc.toprettyxml())
        file.close()
        
        return {'FINISHED'}
        
    def invoke(self,context,event):
        wm = context.window_manager
        if True:
            wm.fileselect_add(self)
            return {'RUNNING_MODAL'}
        elif False:
            return wm.invoke_props_popup(self,event)
    
def exporter_func(self,context):
    self.layout.operator(ShapeExporter.bl_idname,text="Shape (.xml)")
    
def register():
    bpy.utils.register_module(__name__)
    bpy.types.INFO_MT_file_export.append(exporter_func)
    
def unregister():
    bpy.utls.unregister_module(__name__)
    bpy.types.INFO_MT_file_export.remove(exporter_func)
    
if __name__ == "__main__":
    register()