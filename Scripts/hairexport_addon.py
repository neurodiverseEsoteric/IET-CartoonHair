bl_info = {
"name":"Timothy Costigan <costigt@tcd.ie>",
"version":(0,1),
"blender":(2,6,6),
"location":"",
"description":"Iterates through hair strand polylines and saves them to an xml representing the hair",
"warning":"",
"wiki_url":"",
"tracker_url":"",
"category":"Import-Export"}

import bpy
from bpy_extras.io_utils import ExportHelper
from xml.dom.minidom import Document

class HairExporter(bpy.types.Operator,ExportHelper):
    bl_idname = "export_hair.xml"
    bl_label = "Export to hair .xml"
    
    filename_ext = ".xml"
    
    def execute(self,context):
        filepath = self.filepath
        filepath = bpy.path.ensure_ext(filepath,self.filename_ext)
        
        #PERFORM ACTION HERE
        
        doc = Document()
        
        #CREATE DOCUMENT
        
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
    #self.layout.operator(HairExporter.bl_idname,text = "