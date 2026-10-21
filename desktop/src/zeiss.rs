use std::path::Path;
use tauri_plugin_dialog::DialogExt;

// Read directory entries only: ZEN can still be writing the image when motion starts.
// Existing images and changes to their contents must not trigger an experiment.
fn czi_files(folder: &Path) -> Result<Vec<String>, String> {
    let scan = || -> std::io::Result<Vec<String>> {
        let mut files = Vec::new();
        for entry in std::fs::read_dir(folder)? {
            let entry = entry?;
            if entry
                .path()
                .extension()
                .is_some_and(|ext| ext.eq_ignore_ascii_case("czi"))
                && entry.file_type()?.is_file()
            {
                files.push(entry.file_name().to_string_lossy().into_owned());
            }
        }
        Ok(files)
    };
    scan().map_err(|e| format!("Could not read ZEN image folder {}: {e}", folder.display()))
}

#[tauri::command]
pub async fn zeiss_czi_files(folder: String) -> Result<Vec<String>, String> {
    if folder.trim().is_empty() {
        return Err("Choose the folder where ZEN saves .czi images".into());
    }
    tauri::async_runtime::spawn_blocking(move || czi_files(Path::new(&folder)))
        .await
        .map_err(|e| e.to_string())?
}

#[tauri::command]
pub async fn zeiss_choose_folder(window: tauri::WebviewWindow) -> Result<Option<String>, String> {
    tauri::async_runtime::spawn_blocking(move || {
        window
            .dialog()
            .file()
            .set_parent(&window)
            .set_title("Choose Zeiss ZEN image folder")
            .blocking_pick_folder()
            .map(|path| {
                path.into_path()
                    .map(|p| p.to_string_lossy().into_owned())
                    .map_err(|e| e.to_string())
            })
            .transpose()
    })
    .await
    .map_err(|e| e.to_string())?
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scan_includes_only_direct_czi_files_even_while_empty_or_being_written() {
        let folder = std::env::temp_dir().join(format!("teensy-zen-{}", std::process::id()));
        std::fs::create_dir(&folder).unwrap();
        let result = (|| {
            std::fs::write(folder.join("old.czi"), "existing").unwrap();
            std::fs::write(folder.join("new.CZI"), "").unwrap();
            std::fs::write(folder.join("image.czi.tmp"), "temporary").unwrap();
            std::fs::create_dir(folder.join("directory.czi")).unwrap();
            std::fs::write(folder.join("directory.czi/nested.czi"), "nested").unwrap();
            let mut files = czi_files(&folder).unwrap();
            files.sort();
            assert_eq!(files, ["new.CZI", "old.czi"]);
            assert!(czi_files(&folder.join("missing")).is_err());
            assert!(czi_files(&folder.join("old.czi")).is_err());
        });
        let result = std::panic::catch_unwind(result);
        std::fs::remove_dir_all(&folder).unwrap();
        result.unwrap();
    }
}
