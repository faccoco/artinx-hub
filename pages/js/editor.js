$(document).ready(() => {
    initEditor();
    initControl();
});

const buttonConfig = [["apply", "applyConfig"], ["write", "writeConfig"], ["restore", "restoreConfig"]];

function initEditor() {
    fetch("/config/content").then(text => text.text()).then(content => {
        content = content.substring(1, content.length - 1);
        let editor = $("<textarea class=\"mdui-textfield-input\" rows=\"114514\" cols=\"30\" placeholder=\"Press restore to reload the config content from file\" id=\"editor-content\">" + content + "\</textarea/\>");
        $("#text-editor").append(editor);
    });
}

function initControl() {
    for (let each of buttonConfig) {
        $("#control-area").append($("<div class=\"mdui-list-item-content\">" + "<button class=\"mdui-btn  mdui-btn-raised  mdui-ripple mdui-center mdui-color-theme-accent\" onclick=\"" + each[1] + "()\">" + each[0] + "</button>" + "</div>"));
    }
    $("#top-bar").append($("<button class=\"mdui-btn mdui-btn-icon\" type=\"button\" onclick=\"window.close()\">\n" +
        "<i class=\"mdui-icon material-icons\">exit_to_app</i>\n" +
        "</button>"))
    $("#logs").append($("<label class=\"mdui-textfield-label\" id=\"cmd-status\">Logs</label>"));
}

function postControlAction(action, config) {
    return fetch("/config/control", {
        method: "POST",
        body: JSON.stringify({
            control: action,
            content: config
        })
    }).then(res => res.text())
        .then(text => text.substring(1, text.length - 1)).catch(e => console.error(e));
}

function applyConfig() {
    postControlAction("ApplyConfig", $("#editor-content").val()).then(result => {
        $("#cmd-status").text("ApplyConfig: " + result);
    });
}

function writeConfig() {
    if (confirm("Are you sure to write the config to file? This will overwrite the original config file.")) {
        postControlAction("WriteConfig", $("#editor-content").val()).then(result => {
            $("#cmd-status").text("WriteConfig: " + result);
        });
    }
}

function restoreConfig() {
    if (confirm("Are you sure to restore the config from file? This will overwrite the current config in the editor.")) {
        postControlAction("RestoreConfig", "placeholder").then(result => {
            result = result.replaceAll("\\n", "\r\n");
            result = result.replaceAll("\\\"", "\"");
            result = result.replace("\"", "");
            result = result.substring(0, result.lastIndexOf("\""));
            $("#editor-content").val(result);
            document.getElementById("editor-content").value = result;
            $("#cmd-status").text("RestoreConfig: complete");
        });
    }
}

function exitEditor() {
    window.close();
}
