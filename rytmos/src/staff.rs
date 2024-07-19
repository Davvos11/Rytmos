use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::Point,
    image::{Image, ImageRaw},
    pixelcolor::BinaryColor,
    prelude::Size,
    primitives::{Ellipse, Line, Primitive, PrimitiveStyle, Rectangle, StyledDrawable},
    Drawable,
};
use heapless::{FnvIndexMap, Vec};
use log::{debug, error};

use crate::symbols::{
    self, BASS_CLEF, DOTTED_EIGHTH_REST, DOTTED_HALF_REST, DOTTED_QUARTER_REST, EIGHTH_REST,
    EIGHT_FLAG, EMPTY_NOTEHEAD, FILLED_NOTEHEAD, HALF_REST, QUARTER_REST, SIXTEENTH_FLAG,
    SIXTEENTH_REST, WHOLE_REST,
};

#[derive(Debug)]
pub enum EngraveError {
    TooManyMusicSymbolsForVecAllocation,
    MoreMusicSymbolsThanSpacedSymbolsAccountedFor,
    NotEnoughSpaceForSymbols,
    Impossible,
    BeatMapInsertError,
}

#[derive(Clone, Copy)]
pub enum StaffElement<'a> {
    Music(&'a [Music]),
    Barline,
    KeySignature(Key),
    Clef(Clef),
}

#[derive(Clone, Copy)]
pub enum Music {
    Note(Note, Duration),
    Rest(Duration),
    Tie,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StemDirection {
    Up,
    Down,
    NotApplicable,
}

#[derive(Debug, Copy, Clone)]
struct MusicSymbol {
    y: i32,
    stem_direction: StemDirection,
    stem_length: i32,
    kind: Duration,
    rest: bool,
}

#[derive(Debug)]
struct MusicSymbolDefinitions {
    symbols: Vec<MusicSymbol, 16>,
}

impl MusicSymbolDefinitions {
    const REST_OFFSET: i32 = 14;
    const DEFAULT_STEM_LENGTH: i32 = 7;

    fn new(music: &[Music]) -> Result<Self, EngraveError> {
        let mut symbols = Vec::new();

        for &symbol in music {
            match symbol {
                Music::Note(note, duration) => {
                    let head = MusicSymbol {
                        y: note.y_offset(),
                        stem_direction: note.default_stem_direction(),
                        stem_length: Self::DEFAULT_STEM_LENGTH,
                        kind: duration,
                        rest: false,
                    };
                    symbols
                        .push(head)
                        .map_err(|_| EngraveError::TooManyMusicSymbolsForVecAllocation)?;
                }
                Music::Rest(duration) => {
                    let head = MusicSymbol {
                        y: Self::REST_OFFSET,
                        stem_direction: StemDirection::NotApplicable,
                        stem_length: Self::DEFAULT_STEM_LENGTH,
                        kind: duration,
                        rest: true,
                    };
                    symbols
                        .push(head)
                        .map_err(|_| EngraveError::TooManyMusicSymbolsForVecAllocation)?;
                }
                Music::Tie => (),
            }
        }

        Ok(Self { symbols })
    }
}

#[derive(Debug)]
struct SpacedMusicSymbol {
    symbol: MusicSymbol,
    space: i32,
}

#[derive(Debug)]
struct SpacedMusicSymbols {
    symbols: Vec<SpacedMusicSymbol, 16>,
}

impl SpacedMusicSymbols {
    pub const MINIMUM_SYMBOL_SPACE: i32 = 7;
    fn new(symbols: MusicSymbolDefinitions, width: i32) -> Result<Self, EngraveError> {
        let mut spaced_symbols = Vec::new();

        let n = symbols.symbols.len() as i32;
        let base_space = width / n;
        let extra_space = width % n;
        debug!("{} {} {}", extra_space, width, n);
        let spaced = if extra_space > 0 {
            n / extra_space
        } else {
            width
        };

        for (index, symbol) in symbols.symbols.into_iter().enumerate() {
            let space = if index as i32 % spaced == 0 {
                base_space + 1
            } else {
                base_space
            };

            if space < Self::MINIMUM_SYMBOL_SPACE {
                return Err(EngraveError::NotEnoughSpaceForSymbols);
            }

            let spaced_symbol = SpacedMusicSymbol { symbol, space };
            spaced_symbols
                .push(spaced_symbol)
                .map_err(|_| EngraveError::MoreMusicSymbolsThanSpacedSymbolsAccountedFor)?;
        }

        Ok(Self {
            symbols: spaced_symbols,
        })
    }
}

/// Collection of symbols that should be rendered as a single glyph,
/// e.g. a single note, a single rest, or a group of beamed notes.
#[derive(Debug)]
struct GlyphDefinition {
    symbols: Vec<SpacedMusicSymbol, 4>,
    beamed: bool,
}

#[derive(Debug)]
struct Glyphs {
    glyphs: Vec<GlyphDefinition, 16>,
}

impl Glyphs {
    fn new(symbols: SpacedMusicSymbols) -> Result<Self, EngraveError> {
        let mut beat_tracker = 0.;

        // assign a beat to each symbol
        let mut beat_map: FnvIndexMap<u32, Vec<SpacedMusicSymbol, 4>, 16> = FnvIndexMap::new();

        for symbol in symbols.symbols.into_iter() {
            let duration_value = symbol.symbol.kind.value();

            let entry = beat_map.entry(beat_tracker as u32);

            match entry {
                heapless::Entry::Occupied(mut o) => {
                    o.get_mut()
                        .push(symbol)
                        .map_err(|_| EngraveError::BeatMapInsertError)?;
                }
                heapless::Entry::Vacant(e) => {
                    let mut new_vec = Vec::new();
                    new_vec
                        .push(symbol)
                        .map_err(|_| EngraveError::BeatMapInsertError)?;
                    e.insert(new_vec)
                        .map_err(|_| EngraveError::BeatMapInsertError)?;
                }
            }

            beat_tracker += duration_value;
        }

        let mut glyphs: Vec<(u32, Vec<GlyphDefinition, 16>), 4> = Vec::new();

        for (idx, beat) in beat_map.into_iter() {
            let beat_glyphs = Self::beat_to_glyphs(beat)?;
            glyphs
                .push((idx, beat_glyphs))
                .map_err(|_| EngraveError::NotEnoughSpaceForSymbols)?;
        }

        glyphs.sort_by_key(|(idx, _)| *idx);

        let mut glyphs: Vec<GlyphDefinition, 16> =
            glyphs.into_iter().flat_map(|(_, v)| v).collect();

        for glyph in glyphs.iter_mut() {
            Self::fix_stems(glyph);
        }

        Ok(Self { glyphs })
    }

    fn beat_to_glyphs(
        beat: Vec<SpacedMusicSymbol, 4>,
    ) -> Result<Vec<GlyphDefinition, 16>, EngraveError> {
        let mut glyphs = Vec::new();

        let mut glyph_symbols: Vec<SpacedMusicSymbol, 4> = Vec::new();
        for symbol in beat.into_iter() {
            let glyph_is_notes =
                !glyph_symbols.is_empty() && !glyph_symbols.first().unwrap().symbol.rest;

            let symbol_is_note = !symbol.symbol.rest;

            if glyph_is_notes && symbol_is_note {
                // if we're doing notes and this is a note, add it to the symbols and continue
                glyph_symbols
                    .push(symbol)
                    .map_err(|_| EngraveError::NotEnoughSpaceForSymbols)?;
            } else if glyph_is_notes && !symbol_is_note {
                // if we're doing notes and this is a rest, add a glyph and clear glyph_symbols, then add the rest glyph
                // Manual drain implementation
                let mut moved_symbols = Vec::new();
                while !glyph_symbols.is_empty() {
                    moved_symbols
                        .push(glyph_symbols.remove(0))
                        .map_err(|_| EngraveError::Impossible)?;
                }
                // Push the previous glyph
                let beamed = moved_symbols.len() > 1;
                glyphs
                    .push(GlyphDefinition {
                        symbols: moved_symbols,
                        beamed,
                    })
                    .map_err(|_| EngraveError::NotEnoughSpaceForSymbols)?;
                // Push a rest glyph
                let mut rest_symbol = Vec::new();
                rest_symbol
                    .push(symbol)
                    .map_err(|_| EngraveError::NotEnoughSpaceForSymbols)?;
                glyphs
                    .push(GlyphDefinition {
                        symbols: rest_symbol,
                        beamed: false,
                    })
                    .map_err(|_| EngraveError::NotEnoughSpaceForSymbols)?;
            } else if glyph_symbols.is_empty() {
                if symbol.symbol.rest {
                    let mut rest_symbol = Vec::new();
                    rest_symbol
                        .push(symbol)
                        .map_err(|_| EngraveError::NotEnoughSpaceForSymbols)?;
                    glyphs
                        .push(GlyphDefinition {
                            symbols: rest_symbol,
                            beamed: false,
                        })
                        .map_err(|_| EngraveError::NotEnoughSpaceForSymbols)?;
                } else {
                    glyph_symbols
                        .push(symbol)
                        .map_err(|_| EngraveError::NotEnoughSpaceForSymbols)?;
                }
            } else {
                debug!(
                    "{} {} {} {} {:?}",
                    glyph_is_notes,
                    symbol_is_note,
                    !glyph_symbols.is_empty(),
                    !glyph_symbols.first().unwrap().symbol.rest,
                    &glyph_symbols
                );

                panic!("Forgot a case");
            }
        }

        if !glyph_symbols.is_empty() {
            let beamed = glyph_symbols.len() > 1;
            glyphs
                .push(GlyphDefinition {
                    symbols: glyph_symbols,
                    beamed,
                })
                .map_err(|_| EngraveError::NotEnoughSpaceForSymbols)?;
        }

        Ok(glyphs)
    }

    // TODO: unwraps in this function...
    fn fix_stems(glyph: &mut GlyphDefinition) -> &mut GlyphDefinition {
        if glyph.symbols.len() == 1 {
            return glyph;
        }

        // Set the direction of the stems to the same value for each
        let mut stem_directions: Vec<StemDirection, 4> = Vec::new();
        for symbol in glyph.symbols.iter_mut() {
            stem_directions.push(symbol.symbol.stem_direction).unwrap();
        }

        let ups = stem_directions
            .iter()
            .filter(|&&d| d == StemDirection::Up)
            .count();

        let downs = stem_directions
            .iter()
            .filter(|&&d| d == StemDirection::Down)
            .count();

        let direction = if ups >= downs {
            StemDirection::Up
        } else {
            StemDirection::Down
        };

        for symbol in glyph.symbols.iter_mut() {
            symbol.symbol.stem_direction = direction;
        }

        // Set the stem lengths such that they line up
        let stem_offset = if direction == StemDirection::Up {
            -1
        } else {
            1
        };
        let stem_endpoints: Vec<i32, 4> = glyph
            .symbols
            .iter()
            .map(|s| s.symbol.y + stem_offset * MusicSymbolDefinitions::DEFAULT_STEM_LENGTH)
            .collect();

        let most_extreme_endpoint = if direction == StemDirection::Up {
            stem_endpoints.iter().min().unwrap()
        } else {
            stem_endpoints.iter().max().unwrap()
        };

        for symbol in glyph.symbols.iter_mut() {
            symbol.symbol.stem_length = stem_offset * (most_extreme_endpoint - symbol.symbol.y);
        }

        glyph
    }
}

impl Music {
    /// Draws notes, rests, ties, and other music defining notation.
    /// Assumes a beat consists of a quarter note.
    /// x Makes a collection of noteheads that should be rendered
    /// x Calculates the free space that nodeheads can maximally move to the left and right
    ///     x This is the range that the notehead can exist in
    /// x Calculates the Y drawing position of the notehead (relative to the given position)
    /// x Assert that all ranges are at least 7 pixels
    /// x Determines which notes should be tied using the following criteria:
    ///     x The notes must start in the same beat (assumes 4/4, and that vec start = bar start)
    ///     x The notes are eighth, dotted eigtht, or sixteenth duration
    ///     x The notes are consecutive
    ///     x Does NOT take tie-ing into account, any group of notes that follow these criteria are beamed
    /// x This results in a list of glyphs, each containing a vec of symbols
    ///     with their available space. If the Vec contains more than one note, it is beamed
    /// - Actually performs the drawing of the glyps, here finally the position argument is applied to everything:
    ///     - For each notehead, compute the necessary space:
    ///         - whole, half, and quarter notes don't need space for flags
    ///         - Beamed notes may need some space dependending on their position and orientation
    ///     - Move noteheads around to provide necessary space
    ///     - Notes without beams can be drawn trivially using their symbol (with stem in correct direction)
    ///     - Beamed notes need 4 drawing steps
    ///         - 1) Draw the noteheads
    ///         - Determine the direction of all stems (follow majority)
    ///         - Determine the y position of the beam (all beams are strictly horizontal)
    ///         - 2) Draw all stems starting at the notehead up (down) to the computed y
    ///         - 3) Draw a horizontal line for the beam
    ///         - 4) Draw additional horizontal lines for sixteenths
    ///     - Rests are drawn using their symbol at a specific hard coded y position
    /// TODO: ties?
    pub fn draw<D>(
        target: &mut D,
        position: Point,
        width: i32,
        music: &[Music],
    ) -> Result<u32, D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        // TODO: fix all the unwraps by drawing different things
        let symbols = MusicSymbolDefinitions::new(music)
            .and_then(|symbols| SpacedMusicSymbols::new(symbols, width))
            .and_then(Glyphs::new);

        if symbols.is_err() {
            error!("Error engraving symbols: {:?}", symbols);
        }

        let glyphs = symbols.unwrap();

        debug!("{:#?}", glyphs);

        let mut x = 0;
        for glyph in glyphs.glyphs.iter() {
            let beamed = glyph.symbols.len() > 1;
            let glyph_start_x = x;

            for symbol in glyph.symbols.iter() {
                Self::draw_spaced_music_symbol(target, position, x, symbol, beamed)?;

                x += symbol.space;
            }

            if beamed {
                Self::draw_beams(target, position, glyph_start_x, glyph)?;
            }
        }

        Ok(0)
    }

    fn draw_spaced_music_symbol<D>(
        target: &mut D,
        position: Point,
        x: i32,
        symbol: &SpacedMusicSymbol,
        beamed: bool,
    ) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        if symbol.symbol.rest {
            let pos = position + Point::new(x, MusicSymbolDefinitions::REST_OFFSET);
            match symbol.symbol.kind {
                Duration::Eighth => crate::symbols::draw_symbol(target, pos, EIGHTH_REST)?,
                Duration::Whole => crate::symbols::draw_symbol(target, pos, WHOLE_REST)?,
                Duration::Half => crate::symbols::draw_symbol(target, pos, HALF_REST)?,
                Duration::DottedHalf => crate::symbols::draw_symbol(target, pos, DOTTED_HALF_REST)?,
                Duration::Quarter => crate::symbols::draw_symbol(target, pos, QUARTER_REST)?,
                Duration::DottedQuarter => {
                    crate::symbols::draw_symbol(target, pos, DOTTED_QUARTER_REST)?
                }
                Duration::DottedEighth => {
                    crate::symbols::draw_symbol(target, pos, DOTTED_EIGHTH_REST)?
                }
                Duration::Sixteenth => crate::symbols::draw_symbol(target, pos, SIXTEENTH_REST)?,
            };
        } else {
            let position = position + Point::new(x, symbol.symbol.y);
            let line_style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
            let bg_style = PrimitiveStyle::with_stroke(BinaryColor::Off, 1);

            match symbol.symbol.kind {
                Duration::Whole | Duration::Half => {
                    crate::symbols::draw_symbol(target, position, EMPTY_NOTEHEAD)?
                }
                _ => crate::symbols::draw_symbol(target, position, FILLED_NOTEHEAD)?,
            };

            if symbol.symbol.kind != Duration::Whole {
                // Draw stem
                let x_offset = Point::new(1, 0);

                let (start_pos, end_pos) = if symbol.symbol.stem_direction == StemDirection::Up {
                    (
                        position + Point::new(4, 0),
                        position + Point::new(4, -symbol.symbol.stem_length + 1),
                    )
                } else {
                    (
                        position + Point::new(1, 4),
                        position + Point::new(1, symbol.symbol.stem_length + 3),
                    )
                };

                Line::new(start_pos, end_pos)
                    .into_styled(line_style)
                    .draw(target)?;

                Line::new(start_pos + x_offset, end_pos + x_offset)
                    .into_styled(bg_style)
                    .draw(target)?;
            }

            // Draw dot, if applicable
            if matches!(
                symbol.symbol.kind,
                Duration::DottedEighth | Duration::DottedQuarter | Duration::DottedHalf
            ) {
                // Draw two dots, one overlaps with a ledger line
                Rectangle::new(
                    position + Point::new(6, 2),
                    Size {
                        width: 1,
                        height: 1,
                    },
                )
                .into_styled(line_style)
                .draw(target)?;

                Rectangle::new(
                    position + Point::new(6, 0),
                    Size {
                        width: 1,
                        height: 1,
                    },
                )
                .into_styled(line_style)
                .draw(target)?;
            }

            if !beamed {
                // Draw flag
                let flipped = symbol.symbol.stem_direction == StemDirection::Down;

                let offset = if !flipped {
                    Point {
                        x: 6,
                        y: -MusicSymbolDefinitions::DEFAULT_STEM_LENGTH + 1,
                    }
                } else {
                    Point {
                        x: 0,
                        y: MusicSymbolDefinitions::DEFAULT_STEM_LENGTH,
                    }
                };

                let pos = position + offset;

                match symbol.symbol.kind {
                    Duration::Eighth => {
                        crate::symbols::draw_symbol_with_direction(
                            target, pos, EIGHT_FLAG, flipped,
                        )?;
                    }
                    Duration::DottedEighth => {
                        crate::symbols::draw_symbol_with_direction(
                            target,
                            pos,
                            DOTTED_EIGHTH_REST,
                            flipped,
                        )?;
                    }
                    Duration::Sixteenth => {
                        crate::symbols::draw_symbol_with_direction(
                            target,
                            pos,
                            SIXTEENTH_FLAG,
                            flipped,
                        )?;
                    }
                    _ => {}
                }
            }
        }

        Ok(())
    }

    fn draw_beams<D>(
        target: &mut D,
        position: Point,
        x: i32,
        glyph: &GlyphDefinition,
    ) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        if glyph.symbols.is_empty() {
            // TODO: draw a failure
            error!("Empty glyph.");
            return Ok(());
        }

        let style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);

        let first_symbol = glyph.symbols.first().unwrap().symbol;
        let last_symbol = glyph.symbols.first().unwrap().symbol;

        // always draw top beam
        let top_beam_start = position
            + if first_symbol.stem_direction == StemDirection::Up {
                Point::new(x + 4, first_symbol.y - first_symbol.stem_length + 1)
            } else {
                Point::new(x + 1, first_symbol.y + first_symbol.stem_length + 2)
            };
        let beam_length = 8;
        Rectangle::new(
            top_beam_start,
            Size {
                width: beam_length,
                height: 2,
            },
        )
        .draw_styled(&style, target)?;
        // for each consecutive note pair
        // if sixteenths - eight: draw block facing right
        // if eight - sixteenth: draw block facing left
        // if sixteenth - sixteenht: connect

        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Duration {
    Whole,
    Half,
    DottedHalf,
    Quarter,
    DottedQuarter,
    Eighth,
    DottedEighth,
    Sixteenth,
}

impl Duration {
    pub fn value(self) -> f64 {
        match self {
            Duration::Whole => 4.0,
            Duration::Half => 2.0,
            Duration::DottedHalf => 3.0,
            Duration::Quarter => 1.0,
            Duration::DottedQuarter => 1.5,
            Duration::Eighth => 0.5,
            Duration::DottedEighth => 0.75,
            Duration::Sixteenth => 0.25,
        }
    }
}

#[derive(Clone, Copy)]
pub enum Accidental {
    Natural,
    Sharp,
    Flat,
    DoubleSharp,
    DoubleFlat,
}

#[derive(Clone, Copy)]
pub enum Note {
    A(Accidental, i32),
    B(Accidental, i32),
    C(Accidental, i32),
    D(Accidental, i32),
    E(Accidental, i32),
    F(Accidental, i32),
    G(Accidental, i32),
}

impl Note {
    const STEM_DIRECTION_SWITCH_HEIGHT: i32 = 17;

    fn y_offset(self) -> i32 {
        match self {
            Note::A(_, octave) => 38 - octave * 14,
            Note::B(_, octave) => 36 - octave * 14,
            Note::C(_, octave) => 48 - octave * 14,
            Note::D(_, octave) => 46 - octave * 14,
            Note::E(_, octave) => 44 - octave * 14,
            Note::F(_, octave) => 42 - octave * 14,
            Note::G(_, octave) => 40 - octave * 14,
        }
    }

    fn default_stem_direction(&self) -> StemDirection {
        if self.y_offset() > Self::STEM_DIRECTION_SWITCH_HEIGHT {
            StemDirection::Up
        } else {
            StemDirection::Down
        }
    }
}

#[derive(Clone, Copy)]
pub enum Key {
    CMajor,
    FMajor,
    BbMajor,
    EbMajor,
    AbMajor,
    DbMajor,
    GbMajor,
    FsMajor,
    BMajor,
    EMajor,
    AMajor,
    DMajor,
    GMajor,
}

#[derive(Clone, Copy)]
pub enum Clef {
    Bass,
    Treble,
}

impl Clef {
    pub fn draw<D>(self, target: &mut D, position: Point) -> Result<u32, D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        match self {
            Clef::Bass => {
                crate::symbols::draw_symbol(target, position, BASS_CLEF)?;
                Ok(13)
            }
            Clef::Treble => Ok(0),
        }
    }
}

#[derive(Clone, Copy)]
pub struct Staff {
    width: i32,
    position: Point,
}

impl Staff {
    const LINE_SPACING: i32 = 4;
    const LEDGER_MARGIN: i32 = 4 * 3;
    const BASS_CLEF_OFFSET: Point = Point { x: 0, y: 12 };

    pub fn new(width: u32, position: Point) -> Self {
        Self {
            width: width as i32,
            position,
        }
    }

    pub fn draw<D>(&self, target: &mut D, elements: &[StaffElement]) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        let line_style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);

        for i in 0..5 {
            Line::new(
                Point::new(
                    self.position.x,
                    Self::LEDGER_MARGIN + self.position.y + Self::LINE_SPACING * i,
                ),
                Point::new(
                    self.position.x + self.width,
                    Self::LEDGER_MARGIN + self.position.y + Self::LINE_SPACING * i,
                ),
            )
            .into_styled(line_style)
            .draw(target)?;
        }

        let mut working_position = self.position;

        for element in elements {
            let width_used = match element {
                StaffElement::Barline => Self::draw_barline(target, working_position)?,
                StaffElement::KeySignature(_) => todo!(),
                StaffElement::Clef(clef) => {
                    clef.draw(target, working_position + Self::BASS_CLEF_OFFSET)?
                }
                StaffElement::Music(music) => Music::draw(
                    target,
                    working_position,
                    self.width - working_position.x,
                    music,
                )?,
            };

            working_position.x += width_used as i32;
        }

        Ok(())
    }

    fn draw_barline<D>(target: &mut D, working_position: Point) -> Result<u32, D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        Rectangle::new(
            Point::new(working_position.x + 1, 12),
            Size {
                width: 1,
                height: 17,
            },
        )
        .draw_styled(&PrimitiveStyle::with_stroke(BinaryColor::On, 1), target)?;

        Ok(3)
    }
}
