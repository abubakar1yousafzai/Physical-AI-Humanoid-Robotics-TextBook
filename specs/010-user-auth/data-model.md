# Data Model: User Authentication

## Entities

### User
Represents a registered user of the platform.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | UUID | Yes | Primary Key |
| `email` | String | Yes | Unique email address |
| `hashed_password` | String | Yes | Bcrypt hashed password |
| `name` | String | No | Display name |
| `is_active` | Boolean | Yes | Account status (default: true) |
| `is_superuser` | Boolean | Yes | Admin privileges (default: false) |
| `is_verified` | Boolean | Yes | Email verification status (default: false) |
| `created_at` | DateTime | Yes | Account creation timestamp |

### UserProgress
Tracks the chapters completed by a user.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | UUID | Yes | Primary Key |
| `user_id` | UUID | Yes | Foreign Key -> User.id |
| `chapter_id` | String | Yes | ID of the chapter (e.g., "module-01/chapter-01") |
| `completed` | Boolean | Yes | Completion status |
| `completed_at` | DateTime | Yes | When the chapter was completed |

### UserBookmark
Stores chapters bookmarked by a user for later reference.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | UUID | Yes | Primary Key |
| `user_id` | UUID | Yes | Foreign Key -> User.id |
| `chapter_id` | String | Yes | ID of the chapter |
| `created_at` | DateTime | Yes | When the bookmark was created |

## Relationships

- **User** 1:N **UserProgress** (One user has many progress records)
- **User** 1:N **UserBookmark** (One user has many bookmarks)
- **User** 1:N **Conversation** (One user can have many chat conversations - `conversations` table update)
