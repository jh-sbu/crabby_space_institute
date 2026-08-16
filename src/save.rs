use anyhow::{Context, Result, bail};
use directories::ProjectDirs;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

use crate::model::{CraftBlueprint, Vessel};
use crate::simulation::{MissionProgress, SimulationClock};

pub const SAVE_SCHEMA: u32 = 1;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuickSave {
    pub schema_version: u32,
    pub vessel: Vessel,
    pub clock: SimulationClock,
    pub mission: MissionProgress,
    pub script_source: String,
    pub script_state: Option<serde_json::Value>,
}

#[derive(Debug, Clone)]
pub struct SaveStore {
    root: PathBuf,
}

impl Default for SaveStore {
    fn default() -> Self {
        let root = ProjectDirs::from("org", "Crabby Space Institute", "Crabby Space Institute")
            .map(|dirs| dirs.data_local_dir().to_path_buf())
            .unwrap_or_else(|| PathBuf::from("crabby_space_data"));
        Self { root }
    }
}

impl SaveStore {
    #[cfg(test)]
    pub fn at(root: PathBuf) -> Self {
        Self { root }
    }

    pub fn root(&self) -> &Path {
        &self.root
    }

    pub fn ensure(&self) -> Result<()> {
        fs::create_dir_all(self.root.join("crafts"))?;
        fs::create_dir_all(self.root.join("scripts"))?;
        Ok(())
    }

    pub fn quicksave_exists(&self) -> bool {
        self.root.join("quicksave.ron").is_file()
    }

    pub fn save_craft(&self, craft: &CraftBlueprint) -> Result<PathBuf> {
        self.ensure()?;
        let path = self
            .root
            .join("crafts")
            .join(format!("{}.ron", safe_name(&craft.name)));
        let text = ron::ser::to_string_pretty(craft, ron::ser::PrettyConfig::default())?;
        atomic_write(&path, text.as_bytes())?;
        Ok(path)
    }

    pub fn load_craft(&self, name: &str) -> Result<CraftBlueprint> {
        let path = self
            .root
            .join("crafts")
            .join(format!("{}.ron", safe_name(name)));
        let text = fs::read_to_string(&path)
            .with_context(|| format!("Could not read {}", path.display()))?;
        let craft: CraftBlueprint = ron::from_str(&text)?;
        if craft.schema_version != SAVE_SCHEMA {
            bail!("Unsupported craft schema {}", craft.schema_version);
        }
        Ok(craft)
    }

    pub fn list_crafts(&self) -> Vec<String> {
        let Ok(entries) = fs::read_dir(self.root.join("crafts")) else {
            return Vec::new();
        };
        let mut names: Vec<_> = entries
            .flatten()
            .filter_map(|entry| entry.path().file_stem()?.to_str().map(str::to_owned))
            .collect();
        names.sort();
        names
    }

    pub fn save_script(&self, name: &str, source: &str) -> Result<PathBuf> {
        self.ensure()?;
        let path = self
            .root
            .join("scripts")
            .join(format!("{}.lua", safe_name(name)));
        atomic_write(&path, source.as_bytes())?;
        Ok(path)
    }

    pub fn load_script(&self, name: &str) -> Result<String> {
        let path = self
            .root
            .join("scripts")
            .join(format!("{}.lua", safe_name(name)));
        fs::read_to_string(&path).with_context(|| format!("Could not read {}", path.display()))
    }

    pub fn save_quick(&self, save: &QuickSave) -> Result<()> {
        self.ensure()?;
        let text = ron::ser::to_string_pretty(save, ron::ser::PrettyConfig::default())?;
        atomic_write(&self.root.join("quicksave.ron"), text.as_bytes())
    }

    pub fn load_quick(&self) -> Result<QuickSave> {
        let path = self.root.join("quicksave.ron");
        let text = fs::read_to_string(&path)
            .with_context(|| format!("Could not read {}", path.display()))?;
        let save: QuickSave = ron::from_str(&text)?;
        if save.schema_version != SAVE_SCHEMA {
            bail!("Unsupported quicksave schema {}", save.schema_version);
        }
        Ok(save)
    }
}

fn safe_name(name: &str) -> String {
    let cleaned: String = name
        .chars()
        .map(|ch| {
            if ch.is_ascii_alphanumeric() || ch == '-' || ch == '_' {
                ch
            } else {
                '_'
            }
        })
        .collect();
    if cleaned.trim_matches('_').is_empty() {
        "untitled".into()
    } else {
        cleaned
    }
}

fn atomic_write(path: &Path, bytes: &[u8]) -> Result<()> {
    let tmp = path.with_extension("tmp");
    fs::write(&tmp, bytes)?;
    if path.exists() {
        fs::remove_file(path)?;
    }
    fs::rename(&tmp, path)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{PartCatalog, Vessel, stock_craft};

    #[test]
    fn craft_round_trip() {
        let root =
            std::env::temp_dir().join(format!("crabby-space-save-test-{}", std::process::id()));
        let store = SaveStore::at(root.clone());
        let craft = stock_craft();
        store.save_craft(&craft).unwrap();
        let loaded = store.load_craft(&craft.name).unwrap();
        assert_eq!(loaded.parts.len(), craft.parts.len());
        let _ = fs::remove_dir_all(root);
    }

    #[test]
    fn quicksave_round_trip_preserves_flight_and_script_state() {
        let root =
            std::env::temp_dir().join(format!("crabby-space-quick-test-{}", std::process::id()));
        let store = SaveStore::at(root.clone());
        let save = QuickSave {
            schema_version: SAVE_SCHEMA,
            vessel: Vessel::from_blueprint(&stock_craft(), &PartCatalog::default()),
            clock: SimulationClock {
                universal_time: 42.5,
                warp_index: 2,
                paused: false,
            },
            mission: MissionProgress {
                launched: true,
                ..Default::default()
            },
            script_source: "state = { phase = 2 }".into(),
            script_state: Some(serde_json::json!({"phase": 2})),
        };
        store.save_quick(&save).unwrap();
        let loaded = store.load_quick().unwrap();
        assert_eq!(loaded.vessel.primary_body, "carapace");
        assert_eq!(loaded.clock.universal_time, 42.5);
        assert_eq!(loaded.script_state.unwrap()["phase"], 2);
        let _ = fs::remove_dir_all(root);
    }
}
