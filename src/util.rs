
/// Convert Vec<Option<T>> into a bitmask of enabled fields.
///
/// For instance, 
/// 
/// ```
/// use liblinkbotrs::util::vec_to_mask;
/// assert!(vec_to_mask(&vec![Some(0), None, Some(1)]) == 0x05)
/// ```
pub fn vec_to_mask<T>(v: &Vec<Option<T>>) -> u8 {
    let result = v.iter().fold((0, 0), |acc, item| {
        let (mask, i) = acc;
        let new_mask = if item.is_some() {
            mask | (1<<i)
        } else {
            mask
        };
        (new_mask, i+1)
    });
    result.0
}

/// Convert a mask to a Vec of bools
pub fn mask_to_vec(mask: u8) -> Vec<bool>
{
    let mut bools:Vec<bool> = Vec::new();
    for i in 0..8 {
        bools.push( mask&(1<<i) != 0 );
    }
    bools
}

