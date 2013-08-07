function blkStruct = slblocks  
%SLBLOCKS Defines a block library.  
 
% Library's name. The name appears in the Library Browser's  
% contents pane.  
 
blkStruct.Name = ['Water Runner' sprintf('\n') 'Library V2'];  
 
% The function that will be called when the user double-clicks on
% the library's name. 
 
blkStruct.OpenFcn = 'WaterRunner_lib_2';  
 
% The argument to be set as the Mask Display for the subsystem. You
% may comment this line out if no specific mask is desired.  
% Example: blkStruct.MaskDisplay = 'plot([0:2*pi],sin([0:2*pi]));';
 
%blkStruct.MaskDisplay = '';
% End of blocks
