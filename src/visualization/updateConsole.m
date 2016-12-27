function updateConsole(params, string_new)
% todo: add description
% 
% Input:
%  - params(struct) : parameter struct
%  - string_new(string) : new message
%
% Output: none

global gui_handles;

if params.through_gui
    max_n_items = 50;

    string_prev = gui_handles.console_string;
    % todo find '\n' and replace with ''
    string_combined = [string_prev; {[' ',string_new]}];

    string_updated = string_combined(max(1,end-max_n_items):end);

    gui_handles.console_string = string_updated;
    guidata(gui_handles.main_figure, gui_handles);

    console_handle = gui_handles.listbox_console;
    set(console_handle,'String',string_updated);
    index = size(get(console_handle,'string'), 1); % get how many items are in the list box
    set(console_handle,'ListboxTop',index); % set the index of last item to be the index of the top-most string displayed in list box.
else
    fprintf(string_new);
end
