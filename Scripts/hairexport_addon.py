bl_info = {
"name":"Hair Strand Export Addon",
"author":"Timothy Costigan <costigt@tcd.ie>",
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
        
        #PERFORM ACTIONS HERE
        
        #create XML document
        doc = Document()
        
        #need to be in object mode
        bpy.ops.object.mode_set(mode='OBJECT')
        
        #get scale - assuming top of head to base of neck 30cm
        #ensure head is selected or else the scale will be very wrong
        scale = bpy.context.active_object.dimensions.z/0.3
        
        #convert the hair particles to a mesh of edges
        bpy.ops.object.modifier_convert(modifier="ParticleSystem 1")
        
        #get resolution
        resolution = bpy.data.particles["ParticleSettings"].hair_step
        
        #break it up into individual strands
        bpy.ops.mesh.separate(type = 'LOOSE')
        
        #create hair node
        hair = doc.createElement("hair")
        
        #set scale attribute
        hair.setAttribute("scale",str(scale))
        hair.setAttribute("resolution",str(resolution))
        
        #iterate through hair strands
        polylines = bpy.context.selected_objects
        
        #get number of hair strands
        numStrands = len(polylines)
        hair.setAttribute("count",str(numStrands))
        
        for line in polylines:
            #create strand node
            strand = doc.createElement("strand")
            
            #iterate through all the hair vertices
            for vertex in line.data.vertices:
                #create particle node
                particle = doc.createElement("particle")
                #flip axes to xz-y as per ogre standard
                particle.setAttribute("x",str(vertex.co.x))
                particle.setAttribute("y",str(vertex.co.z))
                particle.setAttribute("z",str(-vertex.co.y))
                strand.appendChild(particle)
            
            #add strand to hair
            hair.appendChild(strand)
            
        #append hair to document
        doc.appendChild(hair)
        
        #clean up hair
        bpy.ops.object.delete()
        
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
    self.layout.operator(HairExporter.bl_idname,text = "Hair Strand (.xml)")
    
def register():
    bpy.utils.register_module(__name__)
    bpy.types.INFO_MT_file_export.append(exporter_func)

def unregister():
    bpy.utils.unregister_module(__name__)
    bpy.types.INFO_MT_file_export.remove(exporter_func)

if __name__ == "__main__":
    register()